clc
clear all

syms x y
ezplot(y == (-(x-90)^2)/90 + 80 , [0,90,0,100])