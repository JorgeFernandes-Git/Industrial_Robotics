function M = Trot(phi,theta,psi)

M = rotz(phi)*roty(theta)*rotx(psi);

