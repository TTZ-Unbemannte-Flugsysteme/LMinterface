"""
Web Interface Node for Cocoa Dialogue
Provides a web-based frontend for drone commands via text or speech-to-text.
"""

# Set HuggingFace to offline mode to use cached models (prevents network timeouts)
import os
os.environ['HF_HUB_OFFLINE'] = '1'

import io
import tempfile
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from flask import Flask, render_template, request, jsonify
from faster_whisper import WhisperModel


class WebInterfaceNode(Node):
    """
    ROS2 Node that hosts a web interface for drone commands.
    
    Features:
    - Text input: Type a command and submit
    - Speech-to-text: Record audio in browser, transcribe with Whisper
    - Publishes all commands to /voice/text topic
    """
    
    def __init__(self):
        super().__init__('web_interface_node')
        
        # Declare parameters
        self.declare_parameter('port', 5000)
        self.declare_parameter('whisper_model', 'base.en')
        self.declare_parameter('whisper_device', 'cuda')
        self.declare_parameter('output_topic', '/voice/text')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        whisper_model = self.get_parameter('whisper_model').value
        whisper_device = self.get_parameter('whisper_device').value
        output_topic = self.get_parameter('output_topic').value
        
        # Create publisher
        self.text_publisher = self.create_publisher(String, output_topic, 10)
        
        # Load Whisper model
        self.get_logger().info(f"Loading Whisper model: {whisper_model} on {whisper_device}")
        try:
            compute_type = "int8" if whisper_device == "cpu" else "float16"
            self.whisper_model = WhisperModel(
                whisper_model,
                device=whisper_device,
                compute_type=compute_type,
                cpu_threads=4,
                num_workers=2,
            )
            self.get_logger().info("Whisper model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper on {whisper_device}: {e}")
            self.get_logger().info("Falling back to CPU...")
            self.whisper_model = WhisperModel(
                whisper_model,
                device="cpu",
                compute_type="int8",
                cpu_threads=4,
                num_workers=2,
            )
        
        # Queue for drone responses (thread-safe)
        from collections import deque
        self.response_queue = deque(maxlen=50)  # Keep last 50 responses
        self.response_lock = threading.Lock()
        
        # Subscribe to drone responses
        self.response_sub = self.create_subscription(
            String,
            '/drone_response',
            self._response_callback,
            10
        )
        
        # Setup Flask app
        template_folder = os.path.join(os.path.dirname(__file__), 'templates')
        self.app = Flask(__name__, template_folder=template_folder)
        self._setup_routes()
        
        # Start Flask in a separate thread
        self.flask_thread = threading.Thread(target=self._run_flask, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Web Interface Node Ready")
        self.get_logger().info(f"  Web UI: http://localhost:{self.port}")
        self.get_logger().info(f"  Output Topic: {output_topic}")
        self.get_logger().info(f"  Response Topic: /drone_response")
        self.get_logger().info(f"  Whisper: {whisper_model} on {whisper_device}")
        self.get_logger().info("=" * 50)
    
    def _response_callback(self, msg: String):
        """Handle incoming drone responses."""
        with self.response_lock:
            self.response_queue.append({
                'text': msg.data,
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            })
        self.get_logger().info(f'Received drone response: "{msg.data}"')
    
    def _setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            """Serve the main web page"""
            return render_template('index.html')
        
        @self.app.route('/api/send_text', methods=['POST'])
        def send_text():
            """
            API endpoint to send text directly.
            Expects JSON: {"text": "your command here"}
            """
            try:
                data = request.get_json()
                text = data.get('text', '').strip()
                
                if not text:
                    return jsonify({'success': False, 'error': 'No text provided'}), 400
                
                # Publish the text
                self._publish_text(text)
                
                return jsonify({'success': True, 'text': text})
            
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.app.route('/api/transcribe', methods=['POST'])
        def transcribe():
            """
            API endpoint to transcribe audio.
            Expects audio file in form-data with key 'audio'.
            """
            try:
                if 'audio' not in request.files:
                    return jsonify({'success': False, 'error': 'No audio file provided'}), 400
                
                audio_file = request.files['audio']
                
                # Save to temporary file for Whisper
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                    temp_path = temp_file.name
                    audio_file.save(temp_path)
                
                try:
                    # Transcribe with Whisper
                    text = self._transcribe_audio(temp_path)
                    
                    if text:
                        # Publish the transcribed text
                        self._publish_text(text)
                        return jsonify({'success': True, 'text': text})
                    else:
                        return jsonify({'success': False, 'error': 'No speech detected'}), 400
                
                finally:
                    # Clean up temp file
                    if os.path.exists(temp_path):
                        os.remove(temp_path)
            
            except Exception as e:
                self.get_logger().error(f"Transcription error: {e}")
                return jsonify({'success': False, 'error': str(e)}), 500
        
        @self.app.route('/api/get_responses', methods=['GET'])
        def get_responses():
            """
            API endpoint to get drone responses.
            Returns all responses since the given timestamp (or all if no timestamp).
            """
            try:
                since = float(request.args.get('since', 0))
                
                with self.response_lock:
                    responses = [r for r in self.response_queue if r['timestamp'] > since]
                
                return jsonify({
                    'success': True,
                    'responses': responses
                })
            except Exception as e:
                return jsonify({'success': False, 'error': str(e)}), 500
    
    def _transcribe_audio(self, audio_path):
        """
        Transcribe audio file using Whisper.
        
        Args:
            audio_path: Path to the audio file
            
        Returns:
            Transcribed text or empty string if no speech detected
        """
        self.get_logger().info("Transcribing audio...")
        
        try:
            segments, info = self.whisper_model.transcribe(
                audio_path,
                language="en",
                vad_filter=True,
                vad_parameters={"min_silence_duration_ms": 300},
                beam_size=5,
                best_of=5,
            )
            
            # Collect text from segments
            text_parts = []
            for segment in segments:
                # Filter by confidence (avg_logprob > -0.7 is decent)
                if segment.avg_logprob > -0.7:
                    text_parts.append(segment.text)
            
            text = " ".join(text_parts).strip()
            
            if text:
                self.get_logger().info(f'Transcribed: "{text}"')
            else:
                self.get_logger().info("No speech detected in audio")
            
            return text
        
        except Exception as e:
            self.get_logger().error(f"Whisper transcription failed: {e}")
            return ""
    
    def _publish_text(self, text):
        """
        Publish text to the ROS2 topic.
        
        Args:
            text: Text to publish
        """
        msg = String()
        msg.data = text
        self.text_publisher.publish(msg)
        self.get_logger().info(f'Published to /voice/text: "{text}"')
    
    def _run_flask(self):
        """Run Flask server in a separate thread"""
        # Disable Flask's default logging to reduce noise
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)
        
        self.app.run(
            host='0.0.0.0',
            port=self.port,
            debug=False,
            use_reloader=False,
            threaded=True
        )


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = WebInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
