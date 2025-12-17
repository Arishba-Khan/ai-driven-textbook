from pydantic import BaseModel
from datetime import datetime
from typing import Optional, Dict, Any


class Retrieval(BaseModel):
    id: Optional[int] = None
    query: str
    text_content: str
    source_metadata: Dict[str, Any]
    relevance_score: float
    created_at: datetime