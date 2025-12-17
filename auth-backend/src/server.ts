import express from 'express';
import cors from 'cors';
import { betterAuth } from 'better-auth';
import { toNodeHandler } from 'better-auth/node';
import dotenv from 'dotenv';
import pg from 'pg';

const { Pool } = pg;

// Load environment variables
dotenv.config();

// Initialize Better Auth with custom user fields for background information
const auth = betterAuth({
  database: new Pool({
    connectionString: process.env.DATABASE_URL,
  }),
  secret: process.env.AUTH_SECRET!,
  baseURL: process.env.FRONTEND_URL,
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    autoSignIn: true,
  },
  user: {
    additionalFields: {
      experienceLevel: {
        type: 'string',
        required: false,
      },
      programmingLanguages: {
        type: 'string',
        required: false,
      },
      roboticsExperience: {
        type: 'string',
        required: false,
      },
      gpuAvailable: {
        type: 'boolean',
        required: false,
      },
      hardwareAccess: {
        type: 'string',
        required: false,
      },
    },
  },
});

const app = express();

// Enable CORS for same-origin requests (frontend and backend on same domain in production)
app.use(cors({
  origin: ['http://localhost:3000', 'http://localhost'],
  credentials: true,
}));

console.log('CORS enabled for origins:', ['http://localhost:3000', 'http://localhost']);

// Add Better Auth routes BEFORE body parser
app.all('/api/auth/*', toNodeHandler(auth));

// Parse JSON bodies (after auth routes)
app.use(express.json());

// Health check endpoint
app.get('/api/health', (req, res) => {
  res.status(200).json({ 
    status: 'ok', 
    timestamp: new Date().toISOString(),
    service: 'auth-service' 
  });
});

// Error handling middleware
app.use((err: any, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error('Error in auth server:', err);
  res.status(500).json({ 
    error: 'Internal Server Error',
    message: process.env.NODE_ENV === 'development' ? err.message : 'Something went wrong'
  });
});

const PORT = process.env.PORT || 3001;
app.listen(PORT, () => {
  console.log(`Auth server running on port ${PORT}`);
  console.log(`Health check available at /api/health`);
});

export default app;