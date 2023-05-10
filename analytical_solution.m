function [x,u,J] = analytical_solution(t)
u=[exp(t)/4-1/2];
x=[-3/8*exp(-t)+exp(t)/8-1/2*t+1/4;3/8*exp(-t)+1/8*exp(t)-1/2];
J=0.577678;
end

