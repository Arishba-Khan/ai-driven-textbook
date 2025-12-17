import React, { useState } from 'react';
import { useAuth } from './AuthProvider';

interface FormData {
  email: string;
  password: string;
  name: string;
  experienceLevel: string;
  hardwareAccess: string;
  programmingLanguages: string;
  roboticsExperience: string;
  gpuAvailable: boolean;
}

const SignupForm: React.FC = () => {
  const { signUp } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    name: '',
    experienceLevel: '',
    hardwareAccess: '',
    programmingLanguages: '',
    roboticsExperience: '',
    gpuAvailable: false,
  });
  
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value, type } = e.target;
    const target = e.target as HTMLInputElement; // For checkbox handling

    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? target.checked : value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);

    try {
      await signUp({
        email: formData.email,
        password: formData.password,
        name: formData.name,
        experienceLevel: formData.experienceLevel,
        hardwareAccess: formData.hardwareAccess,
        programmingLanguages: formData.programmingLanguages,
        roboticsExperience: formData.roboticsExperience,
        gpuAvailable: formData.gpuAvailable,
      });
      setSuccess(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during registration');
    } finally {
      setLoading(false);
    }
  };

  if (success) {
    return (
      <div className="alert alert-success" role="alert">
        <h4 className="alert-heading">Registration Successful!</h4>
        <p>You have successfully created your account. You can now access the Physical AI & Humanoid Robotics textbook.</p>
        <hr />
        <p className="mb-0">Check your email for verification if required.</p>
      </div>
    );
  }

  return (
    <div className="card">
      <div className="card-body">
        <h3 className="card-title">Sign Up for Physical AI Textbook</h3>
        
        {error && (
          <div className="alert alert-danger" role="alert">
            {error}
          </div>
        )}
        
        <form onSubmit={handleSubmit}>
          <div className="mb-3">
            <label htmlFor="name" className="form-label">Full Name</label>
            <input
              type="text"
              className="form-control"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleChange}
              required
            />
          </div>
          
          <div className="mb-3">
            <label htmlFor="email" className="form-label">Email Address</label>
            <input
              type="email"
              className="form-control"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
            />
          </div>
          
          <div className="mb-3">
            <label htmlFor="password" className="form-label">Password</label>
            <input
              type="password"
              className="form-control"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              required
              minLength={8}
              placeholder="At least 8 characters with mixed case, number, and special char"
            />
          </div>
          
          <div className="mb-3">
            <label htmlFor="experienceLevel" className="form-label">Experience Level with Robotics</label>
            <select
              className="form-select"
              id="experienceLevel"
              name="experienceLevel"
              value={formData.experienceLevel}
              onChange={handleChange}
              required
            >
              <option value="">Select your experience level</option>
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Experienced">Experienced</option>
            </select>
          </div>
          
          <div className="mb-3">
            <label htmlFor="hardwareAccess" className="form-label">Hardware Access</label>
            <select
              className="form-select"
              id="hardwareAccess"
              name="hardwareAccess"
              value={formData.hardwareAccess}
              onChange={handleChange}
              required
            >
              <option value="">Select your hardware access</option>
              <option value="None">None (Simulation only)</option>
              <option value="Jetson">Jetson Platform</option>
              <option value="Full Robot">Full Robot</option>
              <option value="Lab Access">Lab Access</option>
            </select>
          </div>
          
          <div className="mb-3">
            <label htmlFor="programmingLanguages" className="form-label">Primary Programming Language</label>
            <input
              type="text"
              className="form-control"
              id="programmingLanguages"
              name="programmingLanguages"
              value={formData.programmingLanguages}
              onChange={handleChange}
              required
              placeholder="e.g., Python, C++, JavaScript"
            />
          </div>
          
          <div className="mb-3">
            <label htmlFor="roboticsExperience" className="form-label">Robotics Experience</label>
            <select
              className="form-select"
              id="roboticsExperience"
              name="roboticsExperience"
              value={formData.roboticsExperience}
              onChange={handleChange}
              required
            >
              <option value="">Select your robotics experience</option>
              <option value="None">None</option>
              <option value="Some">Some</option>
              <option value="Experienced">Experienced</option>
            </select>
          </div>
          
          <div className="mb-3 form-check">
            <input
              type="checkbox"
              className="form-check-input"
              id="gpuAvailable"
              name="gpuAvailable"
              checked={formData.gpuAvailable}
              onChange={handleChange}
            />
            <label className="form-check-label" htmlFor="gpuAvailable">
              I have access to a GPU for AI/ML computations
            </label>
          </div>
          
          <button type="submit" className="btn btn-primary" disabled={loading}>
            {loading ? 'Creating Account...' : 'Sign Up'}
          </button>
        </form>
      </div>
    </div>
  );
};

export default SignupForm;