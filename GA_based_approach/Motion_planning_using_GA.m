% Author: Anusha Kilari, Yashwanth Surapaneni, Sailendra karra
% Complex system final project EECE 7065 under Dr. Ali Minai
% This code runs for motion planning for swarm robots
% This code runs for the given environment and obstacles.
% The movement of the swarm robots is depicted using the plot.
clc
clear all
close all
% Defining number of chromosomes,genomes and max number of generations
len=20;                 % The length of the genomes  
popSize=20;             % The size of the population which is number of chromosomes (must be an even number)
maxGens=3000;           % The maximum number of generations allowed in a run
%% Defining obstacles coordinates
Obstacle_1 = [13 13;17 13;17 17;13 17];
Obstacle_2 = [33 38;37 38;37 42;33 42];
Obstacle_3 = [25 20;30 20;30 35;25 35];
% Obstacle_4 = [16 8;18 8;16 10;18 10];
Wall_1     = [0:50;zeros(1,51)]';
Wall_2     = [50*ones(1,50);1:50]';
Wall_3     = [0:49;50*ones(1,50)]'; 
Wall_4     = [zeros(1,49);1:49]';


%% Creating a cell of obstacles & wall
% Obstacles = {Obstacle_1;Obstacle_2;Obstacle_3;Obstacle_4};
Obstacles = {Obstacle_1;Obstacle_2;Obstacle_3};
Wall      = {Wall_1;Wall_2;Wall_3;Wall_4};
%% Creating voronoi points using Voronoi tesselation 
[distances_fin,V_fin,Ind_mins,Ind_ming]=voronoi_tesl(Obstacles,Wall);

%% Generating an optimum path using Dijkstra's algorithm
Graph=distances_fin;
SourceNode=Ind_mins;
TerminalNode=Ind_ming;
%% Calling Dijkstra's algorithm
Route = Dijkstras( Graph, SourceNode, TerminalNode );
X_path=[5; V_fin(Route,1); 40];
Y_path=[3; V_fin(Route,2); 45];
PATH=[X_path Y_path];
%% Calling Genetic Algorithm using the path obtained from Dijkstra's Algo
Genetic_Algo(PATH,len,popSize,maxGens,Obstacles)
% plot(X_path,Y_path)
% xlim([0 50]);
% ylim([0 50]);




