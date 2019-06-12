function I_s = spatialInertiaTensor(I_com, r_com, m)

    r_comM  =   vecX3D(r_com);
    
    I_O     =   I_com - m*r_comM*r_comM;
    
    I_s     =   [I_O        m*r_comM;
                m*r_comM'   m*eye(3)];

end