from pydantic import BaseModel
from typing import Optional


class ErrorDetails(BaseModel):
    code: str
    message: str
    details: Optional[str] = None


class ErrorResponse(BaseModel):
    error: ErrorDetails