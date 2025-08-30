function CAW = CawCal(psi,CA,tht)
CAW = [cos(CA)*cos(psi),   -sin(psi)*cos(tht) + cos(psi)*sin(CA)*sin(tht),   sin(psi)*sin(tht) + cos(psi)*sin(CA)*cos(tht);
    cos(CA)*sin(psi),   cos(psi)*cos(tht) + sin(psi)*sin(CA)*sin(tht),   -cos(psi)*sin(tht) + sin(psi)*sin(CA)*cos(tht);
    -sin(CA),  cos(CA)*sin(tht),  cos(CA)*cos(tht)];