import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import Link from '@docusaurus/Link';
import styles from './auth.module.css';

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
  const [showPassword, setShowPassword] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement>) => {
    const { name, value, type } = e.target;
    // Type assertion to handle the potential checkbox property
    const target = e.target as HTMLInputElement;

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
      <div className={styles.authCard}>
        <div className={styles.iconHeader}>
           <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"></path>
              <polyline points="22 4 12 14.01 9 11.01"></polyline>
           </svg>
        </div>
        <h1 className={styles.title}>Registration Successful!</h1>
        <p className={styles.subtitle}>
          You have successfully created your account. Check your email for verification.
        </p>
        <Link to="/signin" className={styles.submitButton} style={{display: 'inline-block', textDecoration: 'none'}}>
          Go to Sign In
        </Link>
      </div>
    );
  }

  return (
    <div className={styles.authCard}>
      {/* Icon Header */}
      <div className={styles.iconHeader}>
         <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
          <circle cx="12" cy="12" r="1"></circle>
          <circle cx="12" cy="5" r="1"></circle>
          <circle cx="12" cy="19" r="1"></circle>
          <circle cx="5" cy="12" r="1"></circle>
          <circle cx="19" cy="12" r="1"></circle>
          <circle cx="5" cy="5" r="1"></circle>
          <circle cx="19" cy="5" r="1"></circle>
          <circle cx="5" cy="19" r="1"></circle>
          <circle cx="19" cy="19" r="1"></circle>
        </svg>
      </div>

      <h1 className={styles.title}>Sign Up</h1>
      <p className={styles.subtitle}>Create your account to access the textbook</p>
        
      {error && (
        <div className={styles.errorAlert} role="alert">
          {error}
        </div>
      )}
        
      <form onSubmit={handleSubmit}>
        <div className={styles.formGroup}>
          <div className={styles.inputWrapper}>
             <div className={styles.inputIcon}>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2"></path>
                <circle cx="12" cy="7" r="4"></circle>
              </svg>
             </div>
             <input
              type="text"
              className={styles.authInput}
              id="name"
              name="name"
              placeholder="Full Name"
              value={formData.name}
              onChange={handleChange}
              required
            />
          </div>
        </div>

        <div className={styles.formGroup}>
           <div className={styles.inputWrapper}>
            <div className={styles.inputIcon}>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z"></path>
                <polyline points="22,6 12,13 2,6"></polyline>
              </svg>
            </div>
            <input
              type="email"
              className={styles.authInput}
              id="email"
              name="email"
              placeholder="Email Address"
              value={formData.email}
              onChange={handleChange}
              required
            />
          </div>
        </div>
          
        <div className={styles.formGroup}>
           <div className={styles.inputWrapper}>
             <div className={styles.inputIcon}>
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2"></rect>
                <path d="M7 11V7a5 5 0 0 1 10 0v4"></path>
              </svg>
            </div>
            <input
              type={showPassword ? "text" : "password"}
              className={styles.authInput}
              id="password"
              name="password"
              placeholder="Password"
              value={formData.password}
              onChange={handleChange}
              required
              minLength={8}
            />
            <button 
              type="button" 
              className={styles.eyeButton} 
              onClick={() => setShowPassword(!showPassword)}
              aria-label="Toggle password visibility"
            >
               {showPassword ? (
                 <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                   <path d="M17.94 17.94A10.07 10.07 0 0 1 12 20c-7 0-11-8-11-8a18.45 18.45 0 0 1 5.06-5.94M9.9 4.24A9.12 9.12 0 0 1 12 4c7 0 11 8 11 8a18.5 18.5 0 0 1-2.16 3.19m-6.72-1.07a3 3 0 1 1-4.24-4.24"></path>
                   <line x1="1" y1="1" x2="23" y2="23"></line>
                 </svg>
               ) : (
                 <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                   <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"></path>
                   <circle cx="12" cy="12" r="3"></circle>
                 </svg>
               )}
            </button>
          </div>
        </div>
          
        <div className={styles.formGroup}>
            <select
              className={styles.authInput}
              style={{paddingLeft: '16px'}}
              id="experienceLevel"
              name="experienceLevel"
              value={formData.experienceLevel}
              onChange={handleChange}
              required
            >
              <option value="">Select Experience Level</option>
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Experienced">Experienced</option>
            </select>
        </div>
          
        <div className={styles.formGroup}>
            <select
              className={styles.authInput}
              style={{paddingLeft: '16px'}}
              id="hardwareAccess"
              name="hardwareAccess"
              value={formData.hardwareAccess}
              onChange={handleChange}
              required
            >
              <option value="">Select Hardware Access</option>
              <option value="None">None (Simulation only)</option>
              <option value="Jetson">Jetson Platform</option>
              <option value="Full Robot">Full Robot</option>
              <option value="Lab Access">Lab Access</option>
            </select>
        </div>

        <div className={styles.formGroup}>
            <div className={styles.inputWrapper}>
             <div className={styles.inputIcon}>
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <polyline points="16 18 22 12 16 6"></polyline>
                  <polyline points="8 6 2 12 8 18"></polyline>
                </svg>
             </div>
            <input
              type="text"
              className={styles.authInput}
              id="programmingLanguages"
              name="programmingLanguages"
              placeholder="Primary Prog. Language"
              value={formData.programmingLanguages}
              onChange={handleChange}
              required
            />
            </div>
        </div>

        <div className={styles.formGroup}>
            <select
              className={styles.authInput}
              style={{paddingLeft: '16px'}}
              id="roboticsExperience"
              name="roboticsExperience"
              value={formData.roboticsExperience}
              onChange={handleChange}
              required
            >
              <option value="">Robotics Experience?</option>
              <option value="None">None</option>
              <option value="Some">Some</option>
              <option value="Experienced">Experienced</option>
            </select>
        </div>
          
        <div className="mb-3 form-check" style={{textAlign: 'left', paddingLeft: '32px'}}>
            <input
              type="checkbox"
              className="form-check-input"
              id="gpuAvailable"
              name="gpuAvailable"
              checked={formData.gpuAvailable}
              onChange={handleChange}
            />
            <label className="form-check-label" htmlFor="gpuAvailable" style={{marginLeft: '8px', color: '#666'}}>
              I have access to a GPU
            </label>
        </div>
          
        <button type="submit" className={styles.submitButton} disabled={loading}>
            {loading ? 'Creating Account...' : 'Sign Up'}
        </button>
      </form>

      <p className={styles.footerText}>
        Already have an account? <Link to="/signin" className={styles.footerLink}>Sign In</Link>
      </p>
    </div>
  );
};

export default SignupForm;