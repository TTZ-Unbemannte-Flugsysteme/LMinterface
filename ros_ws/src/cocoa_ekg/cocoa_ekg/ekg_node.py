"""
EKG Node - Embodied Knowledge Graph ROS2 Node
Provides QueryEKG and GetKnowledgeSummary services
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from ament_index_python.packages import get_package_share_directory

from cocoa_msgs.srv import QueryEKG, GetKnowledgeSummary, GetAllObjects
from .knowledge_graph import KnowledgeGraph


class EKGNode(Node):
    """
    Embodied Knowledge Graph ROS2 Node
    
    Stores spatial knowledge about objects and semantic relationships.
    Provides:
      - /ekg/query: Look up individual object positions
      - /ekg/get_knowledge_summary: Get serialized graph for LLM prompt injection
    """
    
    def __init__(self):
        super().__init__('ekg_node')
        
        # Declare parameter for config filename (defaults to objects.yaml)
        self.declare_parameter('config_file', 'objects.yaml')
        config_file_name = self.get_parameter('config_file').get_parameter_value().string_value
        
        # Try to load config from package share directory
        config_path = None
        try:
            pkg_share = get_package_share_directory('cocoa_ekg')
            config_path = os.path.join(pkg_share, 'config', config_file_name)
            self.get_logger().info(f"Loading objects from: {config_path}")
        except Exception as e:
            self.get_logger().warn(f"Could not find config: {e}")
        
        # Initialize the knowledge graph with config path
        self.kg = KnowledgeGraph(config_path=config_path)
        
        # Create QueryEKG service (existing)
        self.query_srv = self.create_service(
            QueryEKG,
            '/ekg/query',
            self.query_ekg_callback
        )
        
        # Create GetKnowledgeSummary service (NEW: for Intent Extractor)
        self.summary_srv = self.create_service(
            GetKnowledgeSummary,
            '/ekg/get_knowledge_summary',
            self.get_knowledge_summary_callback
        )
        
        # Log loaded objects and relationships
        self.get_logger().info("=" * 50)
        self.get_logger().info("EKG Node Ready")
        self.get_logger().info(f"  Objects: {len(self.kg.graph.nodes)}")
        for obj in self.kg.get_all_objects():
            self.get_logger().info(f"    - {obj.name}: {obj.position}")
        self.get_logger().info(f"  Relationships: {len(self.kg.graph.edges)}")
        for rel in self.kg.get_relationships():
            self.get_logger().info(f"    - {rel['from']} is {rel['relation']} {rel['to']}")
        self.get_logger().info("  Services:")
        self.get_logger().info("    - /ekg/query")
        self.get_logger().info("    - /ekg/get_knowledge_summary")
        self.get_logger().info("    - /ekg/get_all_objects")
        self.get_logger().info("=" * 50)
        
        # Create GetAllObjects service (NEW: for LGA collision checking)
        self.all_objects_srv = self.create_service(
            GetAllObjects,
            '/ekg/get_all_objects',
            self.get_all_objects_callback
        )

    def query_ekg_callback(self, request, response):
        """Handle QueryEKG service requests - look up individual objects"""
        object_name = request.object_name
        self.get_logger().info(f'Query received for: "{object_name}"')
        
        obj = self.kg.query_spatial_object(object_name)
        
        if obj:
            response.found = True
            response.position = Point(
                x=float(obj.position[0]),
                y=float(obj.position[1]),
                z=float(obj.position[2])
            )
            response.category = obj.category
            response.error_message = ""
            self.get_logger().info(f'Found: {obj.name} at {obj.position}')
        else:
            response.found = False
            response.position = Point(x=0.0, y=0.0, z=0.0)
            response.category = ""
            response.error_message = f"Object '{object_name}' not found in graph"
            self.get_logger().info(f'Not found: "{object_name}"')
        
        return response

    def get_knowledge_summary_callback(self, request, response):
        """Handle GetKnowledgeSummary service - return serialized graph for LLM"""
        self.get_logger().info("Knowledge summary requested for LLM prompt injection")
        
        response.knowledge_text = self.kg.serialize_for_llm()
        response.object_count = len(self.kg.graph.nodes)
        response.relationship_count = len(self.kg.graph.edges)
        
        self.get_logger().info(
            f"Returning summary: {response.object_count} objects, "
            f"{response.relationship_count} relationships"
        )
        
        return response

    def get_all_objects_callback(self, request, response):
        """Handle GetAllObjects service - return all object positions for collision checking"""
        self.get_logger().info("All objects requested for LGA collision checking")
        
        objects = self.kg.get_all_objects()
        
        response.names = []
        response.positions = []
        response.sizes = []
        response.categories = []
        
        for obj in objects:
            response.names.append(obj.name)
            response.positions.append(Point(
                x=float(obj.position[0]),
                y=float(obj.position[1]),
                z=float(obj.position[2])
            ))
            response.sizes.append(Vector3(
                x=float(obj.size[0]),
                y=float(obj.size[1]),
                z=float(obj.size[2])
            ))
            response.categories.append(obj.category)
        
        response.count = len(objects)
        self.get_logger().info(f"Returning {response.count} objects with sizes")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EKGNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

