"""
Knowledge Graph for Cocoa EKG
Loads objects and semantic relationships from YAML config
Provides serialization for LLM prompt injection
"""

import os
import yaml
import networkx as nx
from dataclasses import dataclass
from typing import Optional, List, Dict
from ament_index_python.packages import get_package_share_directory


@dataclass
class SpatialObject:
    name: str
    position: tuple  # (x, y, z)
    category: str    # e.g. "furniture", "obstacle", "landmark"
    size: tuple = (0.5, 0.5, 0.5)  # (width, depth, height) - default 0.5m cube


class KnowledgeGraph:
    """
    Embodied Knowledge Graph with semantic relationships.
    
    Stores objects as nodes and relationships as directed edges.
    Provides serialization for LLM prompt injection.
    """
    
    def __init__(self, config_path: str = None):
        # DiGraph: Directed graph for semantic relationships (e.g., pallet_1 -> shelf_a)
        self.graph = nx.DiGraph()
        self._config = None  # Store config for relationship loading
        self._load_objects(config_path)

    def _load_objects(self, config_path: str = None):
        """Load objects and relationships from YAML config or use defaults"""
        objects = []
        relationships = []
        
        # Try to load from provided config path
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    self._config = yaml.safe_load(f)
                    objects = self._config.get('objects', [])
                    relationships = self._config.get('relationships', [])
            except Exception as e:
                print(f"Warning: Could not load config: {e}")
        
        # Try package share directory as fallback
        if not objects:
            try:
                pkg_share = get_package_share_directory('cocoa_ekg')
                config_file = os.path.join(pkg_share, 'config', 'objects.yaml')
                if os.path.exists(config_file):
                    with open(config_file, 'r') as f:
                        self._config = yaml.safe_load(f)
                        objects = self._config.get('objects', [])
                        relationships = self._config.get('relationships', [])
            except Exception:
                pass
        
        # Load objects into graph as nodes
        if objects:
            for obj in objects:
                pos = tuple(obj.get('position', [0, 0, 0]))
                size = tuple(obj.get('size', [0.5, 0.5, 0.5]))  # Load size from YAML
                self.add_spatial_object(SpatialObject(
                    name=obj.get('name', 'unknown'),
                    position=pos,
                    category=obj.get('category', 'object'),
                    size=size
                ))
        else:
            # Fallback defaults
            self._load_defaults()
        
        # Load relationships into graph as edges
        for rel in relationships:
            from_obj = rel.get('from')
            to_obj = rel.get('to')
            relation = rel.get('relation', 'related_to')
            if from_obj and to_obj:
                self.add_relationship(from_obj, to_obj, relation)

    def _load_defaults(self):
        """Fallback default objects matching cocoa_demo_warehouse.sdf SPACIOUS layout"""
        # Format: (name, position, category, size)
        defaults = [
            ("shelf_a", (-8.0, 6.0, 1.0), "furniture", (2.5, 0.8, 2.0)),
            ("shelf_b", (8.0, 6.0, 1.0), "furniture", (2.5, 0.8, 2.0)),
            ("pallet_1", (-5.0, 2.0, 0.6), "furniture", (1.3, 1.0, 1.2)),
            ("pallet_2", (5.0, 2.0, 0.6), "furniture", (1.3, 1.0, 1.2)),
            ("center_rack", (0.0, 3.0, 0.5), "obstacle", (0.5, 2.0, 1.0)),
            ("forklift", (5.0, -5.0, 0.6), "obstacle", (1.5, 1.0, 1.2)),
            ("box_1", (-5.0, -5.0, 0.4), "obstacle", (0.5, 0.5, 0.5)),
            ("landing_pad", (0.0, -8.0, 0.01), "landmark", (1.6, 1.6, 0.02)),
        ]
        for name, pos, cat, size in defaults:
            self.add_spatial_object(SpatialObject(name, pos, cat, size))
        
        # Default relationships
        default_relationships = [
            ("pallet_1", "shelf_a", "near"),
            ("pallet_2", "shelf_b", "near"),
        ]
        for from_obj, to_obj, rel in default_relationships:
            self.add_relationship(from_obj, to_obj, rel)

    def add_spatial_object(self, spatial_object: SpatialObject):
        """Add an object to the graph as a node"""
        self.graph.add_node(
            spatial_object.name,
            position=spatial_object.position,
            category=spatial_object.category,
            size=spatial_object.size
        )

    def add_relationship(self, from_obj: str, to_obj: str, relation: str):
        """Add a semantic relationship (directed edge) between two objects"""
        self.graph.add_edge(from_obj, to_obj, relation=relation)

    def query_spatial_object(self, name: str) -> Optional[SpatialObject]:
        """Query an object by name"""
        # Try exact match
        if name in self.graph.nodes:
            data = self.graph.nodes[name]
            return SpatialObject(name, data['position'], data['category'])
        
        # Try case-insensitive match
        name_lower = name.lower()
        for node_name in self.graph.nodes:
            if node_name.lower() == name_lower:
                data = self.graph.nodes[node_name]
                return SpatialObject(node_name, data['position'], data['category'])
        
        return None

    def get_all_objects(self) -> List[SpatialObject]:
        """Get all objects in the graph (only nodes with position/category)"""
        objects = []
        for name, data in self.graph.nodes(data=True):
            # Only include nodes that are actual spatial objects
            if 'position' in data and 'category' in data:
                size = data.get('size', (0.5, 0.5, 0.5))
                objects.append(SpatialObject(name, data['position'], data['category'], size))
        return objects

    def get_relationships(self) -> List[Dict]:
        """Get all relationships as a list of dicts"""
        return [
            {'from': u, 'to': v, 'relation': data.get('relation', 'related_to')}
            for u, v, data in self.graph.edges(data=True)
        ]

    def serialize_for_llm(self) -> str:
        """
        Serialize the knowledge graph to LLM-readable text.
        
        NO REASONING HERE - just dump the data for LLM to interpret.
        Output format is designed to be compact (~60 tokens for 8 objects + 5 relationships).
        
        Returns:
            str: Formatted knowledge graph for prompt injection
        """
        lines = ["ENVIRONMENT KNOWLEDGE:", ""]
        
        # Group objects by category for readability (only nodes with position)
        categories: Dict[str, List[str]] = {}
        for node, data in self.graph.nodes(data=True):
            if 'position' not in data:
                continue  # Skip relationship-only nodes
            cat = data.get('category', 'unknown')
            categories.setdefault(cat, []).append(node)
        
        lines.append("Objects:")
        for cat, objects in sorted(categories.items()):
            obj_str = ", ".join(sorted(objects))
            lines.append(f"  {cat}: {obj_str}")
        
        # Add relationships
        edges = list(self.graph.edges(data=True))
        if edges:
            lines.append("")
            lines.append("Relationships:")
            for u, v, data in edges:
                rel = data.get('relation', 'related_to')
                lines.append(f"  - {u} is {rel} {v}")
        
        return "\n".join(lines)