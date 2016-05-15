clc
clear all

syms x y
ezplot(y == (-(x - 90)^2)/25 + 80 , [0,180,0,100])