import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
from app.core.config import settings


class QdrantService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = settings.qdrant_collection_name
        self.cohere_client = cohere.Client(api_key=settings.cohere_api_key)
    
    def create_collection_if_not_exists(self):
        """
        Creates the collection if it doesn't exist.
        This would typically be run during application startup.
        """
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),  # Cohere embeddings are 1024-dim
            )
    
    def search(self, query_text: str, limit: int = 5) -> List[dict]:
        """
        Embeds the query text and performs a similarity search in the Qdrant collection.
        """
        # Generate embedding for the query text
        response = self.cohere_client.embed(
            texts=[query_text],
            model="embed-multilingual-v3.0",
            input_type="search_query"
        )
        query_embedding = response.embeddings[0]
        
        # Perform the search in Qdrant
        search_results = self.client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=limit,
            with_payload=True,
        ).points
        
        # Format results
        results = []
        for result in search_results:
            results.append({
                "text_content": result.payload.get("text"),
                "source_metadata": result.payload.get("metadata", {}),
                "relevance_score": result.score
            })
        
        return results
    
    def close(self):
        """
        Closes the Qdrant client connection.
        """
        if hasattr(self.client, '_client'):
            self.client.close()


# Global instance
qdrant_service = QdrantService()