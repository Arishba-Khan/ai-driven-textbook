from typing import List, Dict, Any
from app.core.qdrant_service import qdrant_service


class QdrantRetriever:
    def __init__(self):
        self.qdrant_service = qdrant_service

    def retrieve(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant text chunks based on the query using Qdrant vector search.
        
        Args:
            query: The search query
            limit: Maximum number of results to return
            
        Returns:
            List of dictionaries containing text_content, source_metadata, and relevance_score
        """
        return self.qdrant_service.search(query_text=query, limit=limit)