function [f,J, domerr]= mcpfuncjacEval(z,jacflag)
    global contact_normal;  global safe_dist;  global closeset_dist;
    global dof; global v; global h; global q_o; global num_contact;

    f = [];  J = []; domerr = 0;    
    qn = z(1:dof); pn = z(end);    
    % Equation of motion
    f(1:dof) = (q_o - qn) + h*v + h*pn*contact_normal;
    f(dof+num_contact) = -(contact_normal'*(qn -q_o) + (closeset_dist - safe_dist));    
    % Jacobian of Equation of Motion
    if (jacflag)
        J(1, 1) = -1;  J(2, 2) = -1;        
        J(1:2, 3) = h*contact_normal;  J(3, 1:2) = -contact_normal';        
        J=sparse(J);
    end
end
