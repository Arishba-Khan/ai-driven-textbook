import React from 'react';
import { useAuth } from './AuthProvider';
import styles from './auth.module.css';

const NavbarAuth: React.FC = () => {
  const { user, signOut, loading } = useAuth();

  if (loading) {
    return <div className="navbar__item">Loading...</div>;
  }

  if (user) {
    return (
      <div className="navbar__item dropdown dropdown--hoverable dropdown--right">
        <a className="navbar__link dropdown__trigger" href="#account">
          Welcome, {user.name || user.email.split('@')[0]}
        </a>
        <ul className="dropdown__menu">
          <li>
            <button 
              className={styles.signOutButton}
              onClick={(e) => {
                e.preventDefault();
                signOut();
              }}
            >
              Sign Out
            </button>
          </li>
        </ul>
      </div>
    );
  }

  return (
    <div className="navbar__item">
      <div className={styles.navContainer}>
        <a className={styles.navSignIn} href="/signin">Sign In</a>
        <a className={styles.navSignUp} href="/signup">Sign Up</a>
      </div>
    </div>
  );
};

export default NavbarAuth;