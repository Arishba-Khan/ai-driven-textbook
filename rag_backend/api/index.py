import sys
import os
# Add the app directory to the Python path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from app.main import app
from mangum import Mangum


# Create the Mangum adapter for serverless deployment
handler = Mangum(app, api_gateway_base_path="/api/rag")

# For Vercel Python runtime
def main(event, context):
    return handler(event, context)