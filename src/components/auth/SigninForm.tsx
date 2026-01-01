import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import Link from '@docusaurus/Link';
import styles from './auth.module.css';

interface FormData {
  email: string;
  password: string;
}

const SigninForm: React.FC = () => {
  const { signIn } = useAuth();
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
  });
  
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [showPassword, setShowPassword] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);
    setLoading(true);
    
    try {
      await signIn(formData.email, formData.password);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during login');
    } finally {
      setLoading(false);
    }
  };

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

      <h1 className={styles.title}>Sign in to continue</h1>
      <p className={styles.subtitle}>Please sign in to start your learning journey</p>
      
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
                <path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z"></path>
                <polyline points="22,6 12,13 2,6"></polyline>
              </svg>
            </div>
            <input
              type="email"
              className={styles.authInput}
              id="email"
              name="email"
              placeholder="Email"
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
        
        <button type="submit" className={styles.submitButton} disabled={loading}>
          {loading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>
      
      <div className={styles.divider}>
        <span className={styles.dividerText}>Or continue with</span>
      </div>

      <div className={styles.socialGroup}>
        <button className={styles.socialButton} title="Google">
           <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
             <path d="M12.24 10.285V14.4h6.806c-.275 1.765-2.056 5.174-6.806 5.174-4.095 0-7.439-3.389-7.439-7.574s3.345-7.574 7.439-7.574c2.33 0 3.891.989 4.785 1.849l3.254-3.138C18.189 1.186 15.479 0 12.24 0c-6.635 0-12 5.365-12 12s5.365 12 12 12c6.926 0 11.52-4.869 11.52-11.726 0-.788-.085-1.39-.189-1.989H12.24z"/>
           </svg>
        </button>
        <button className={styles.socialButton} title="Facebook">
           <svg width="20" height="20" viewBox="0 0 24 24" fill="currentColor">
              <path d="M9.101 23.691v-7.98H6.627v-3.667h2.474v-1.58c0-4.085 1.848-5.978 5.858-5.978.401 0 .955.042 1.468.103a8.68 8.68 0 0 1 1.141.195v3.325a8.623 8.623 0 0 0-.653-.036c-2.148 0-2.971.956-2.971 3.594v.376h5.513l-1.007 3.667h-4.506v7.98H9.101z"/>
           </svg>
        </button>
         <button className={styles.socialButton} title="Apple">
           <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
              <path d="M17.05 20.28c-.98.95-2.05.88-3.08.4-.96-.4-2-.4-3 0-1.07.45-2.09.43-3.08-.4C5.55 17.84 4 14.28 4 11.2c0-2.9 1.7-4.6 4-4.6 1.07 0 2.18.55 2.8 1.15.5.47 1.05.47 1.55 0 .62-.6 1.73-1.15 2.8-1.15 2.05 0 3.6 1.5 3.96 1.63-.01.07-.4.17-.5.27-1.1.9-1.8 2.22-1.8 3.58s.77 2.72 1.95 3.52c-.75 2.1-1.3 3.25-1.8 3.75zM12 4.8c.03-.02.06-.04.09-.06.77-1.4 1.84-2.28 3.3-2.6.28 1.55-.4 3.05-1.54 4.15-1 .85-2.45.98-3.23.6.1-1.15.65-2.12 1.38-3.09z"/>
           </svg>
        </button>
      </div>

      <p className={styles.footerText}>
        Don't have an account? <Link to="/signup" className={styles.footerLink}>Sign Up</Link>
      </p>
    </div>
  );
};

export default SigninForm;