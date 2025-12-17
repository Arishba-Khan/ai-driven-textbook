from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # API Keys
    groq_api_key: str
    cohere_api_key: str
    qdrant_api_key: str
    qdrant_url: str
    qdrant_collection_name: str = "humanoid_ai_book"
    
    # Database Configuration
    neon_db_url: str
    
    # Optional Configuration
    debug_mode: bool = False
    log_level: str = "INFO"
    
    class Config:
        env_file = ".env"
        case_sensitive = True


# Create a single instance of settings
settings = Settings()