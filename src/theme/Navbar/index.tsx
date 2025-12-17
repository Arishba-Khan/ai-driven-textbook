import React from 'react';
import Navbar from '@theme-original/Navbar';
import NavbarAuth from '@site/src/components/auth/NavbarAuth';

const NavbarWrapper = (props) => {
  return (
    <>
      <Navbar {...props} />
      <NavbarAuth />
    </>
  );
};

export default NavbarWrapper;