import React from 'react';
import { useAuth } from './AuthProvider';

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
              className="dropdown__link"
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
      <div className="navbar__links">
        <a className="navbar__link" href="/signin">Sign In</a>
        <a className="button button--primary button--outline navbar__link" href="/signup">Sign Up</a>
      </div>
    </div>
  );
};

export default NavbarAuth;