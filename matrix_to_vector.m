function[vect]=matrix_to_vector(X)
% This function takes a SE(3) matrix and converts it into a 1*12 row vector.
for i=1:3
    vect(1,i)=X(1,i);
    vect(1,i+3)=X(2,i);
    vect(1,i+6)=X(3,i);
    vect(1,i+9)=X(i,4);
end
end

