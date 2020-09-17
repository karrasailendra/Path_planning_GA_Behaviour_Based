% Author: Anusha Kilari, Yashwanth Surapaneni, Sailendra karra
% Complex system final project EECE 7065 under Dr. Ali Minai
% This code runs genetic algorithm on swarm robots automatically by giving
% the following inputs to len (length of genomes),popSize(number of chromosomes), 
% maxGens(number of generations allowed) and axis_scale(to check in the environment with scale)
% path from the previous function called Dijkstra's Algorithm
% The movement of the swarm robots is depicted using the plot.
function Genetic_Algo(path,len,popSize,maxGens,obstacles)

axis_scale=50;

avgFitnessHist=zeros(1,maxGens+1);
minFitnessHist=zeros(1,maxGens+1);


r=20;                    % radius for initial population generation
% Initial population generation
for m=1:popSize
    pop(m,:,:)=1+r*rand(len,2);
end
% Finding the fitness function values for the initial population
fitnessVals2=fitnessfunc(pop,path(1,:),obstacles);
[fitnessVals3,Ind1]=sort(fitnessVals2);
%% Loop for all the waypoints in a path 
for InterGoals=1:size(path,1)
  % Loop for generations of chromosomes
for gen=0:maxGens 
%% implementation of 1-pt crossover
    % determine 10 parents of each mating pair from the available
    % chromosomes
%     parentIndices=randperm(size(pop,1));
   
    firstParents=(pop(Ind1(1:(2)),:,:));
    secondParents=(pop(Ind1(3:4),:,:));
    ThirdParents=(pop(Ind1(5:6),:,:));
    fourParents=(pop(Ind1(7:8),:,:));
    fiveParents=(pop(Ind1(9:10),:,:));
    sixParents=(pop(Ind1(11:12),:,:));
    sevenParents=(pop(Ind1(13:14),:,:));
    eightParents=(pop(Ind1(15:16),:,:));
    nineParents=(pop(Ind1(17:18),:,:));
    tenParents=(pop(Ind1(19:20),:,:));

    % Generation of Kids from the parents
    
    firstKids=firstParents;
    secondKids=secondParents;
    thirdKids=ThirdParents;
    fourKids=fourParents;
    fiveKids=fiveParents;
    sixKids=sixParents;
    sevenKids=sevenParents;
    eightKids=eightParents;
    nineKids=nineParents;
    tenKids=tenParents;
     
  firstKids(randperm(2,1),randperm(len,1),:)= secondParents(randperm(2,1),randperm(len,1),:);
  secondKids(randperm(2,1),randperm(len,1),:)= firstParents(randperm(2,1),randperm(len,1),:);
  thirdKids(randperm(2,1),randperm(len,1),:)= fourParents(randperm(2,1),randperm(len,1),:);
  fourKids(randperm(2,1),randperm(len,1),:)= ThirdParents(randperm(2,1),randperm(len,1),:);
  fiveKids(randperm(2,1),randperm(len,1),:)= sixParents(randperm(2,1),randperm(len,1),:);
  sixKids(randperm(2,1),randperm(len,1),:)= fiveParents(randperm(2,1),randperm(len,1),:);
  sevenKids(randperm(2,1),randperm(len,1),:)= eightParents(randperm(2,1),randperm(len,1),:);
  eightKids(randperm(2,1),randperm(len,1),:)= sevenParents(randperm(2,1),randperm(len,1),:);
  nineKids(randperm(2,1),randperm(len,1),:)= tenParents(randperm(2,1),randperm(len,1),:);
  tenKids(randperm(2,1),randperm(len,1),:)= nineParents(randperm(2,1),randperm(len,1),:);
  %% Implementation of mutation on the Kids generated from Crossover
   num_mutation=5; % 5-Pt Mutation applied 
   % for x-coordinate
  firstKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
  secondKids(randperm(2,1),randperm(len,num_mutation),1)=path(InterGoals,1)*rand(num_mutation,1);
  thirdKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
  fourKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
  fiveKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
  sixKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
  sevenKids(randperm(2,1),randperm(len,num_mutation),1)=path(InterGoals,1)*rand(num_mutation,1);
  eightKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
  nineKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
  tenKids(randperm(2,1),randperm(len,num_mutation),1)= path(InterGoals,1)*rand(num_mutation,1);
    % for y coordinate
  firstKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
  secondKids(randperm(2,1),randperm(len,num_mutation),2)=path(InterGoals,2)*rand(num_mutation,1);
  thirdKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
  fourKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
  fiveKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
  sixKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
  sevenKids(randperm(2,1),randperm(len,num_mutation),2)=path(InterGoals,2)*rand(num_mutation,1);
  eightKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
  nineKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
  tenKids(randperm(2,1),randperm(len,num_mutation),2)= path(InterGoals,2)*rand(num_mutation,1);
      
 % Appending all the Kids generated from different parents  
kids_mat=[firstKids; secondKids;thirdKids;fourKids;fiveKids;sixKids;sevenKids;eightKids;nineKids;tenKids];
 % Appendiing Kids to original population     
pop=[pop;kids_mat];
%% Calculating the Fitness Function values on the chromosomes generated
fitnessVals=fitnessfunc(pop,path(InterGoals,:),obstacles);
[fitnessVals1,Ind]=sort(fitnessVals);

 %selection of chromosomes which has best fitness values (having minimum fitness values)
    pop1=pop(Ind(1:(popSize)),:,:);
    pop=pop1;
    [minFitnessHist(1,gen+1),minIndex]=min(fitnessVals);  
    avgFitnessHist(1,gen+1)=mean(fitnessVals);       
end % End loop for generations
InterGoals
%% Calculating the average distances travelled by each robot 
if(InterGoals>1)
    for i=1:length(pop2)
        dist_travel(i)=sqrt((pop2(i,1)-squeeze(pop1(1,i,1)))^2+(pop2(i,1)-squeeze(pop1(1,i,1)))^2);
    end
 dist_travel1=dist_travel1+dist_travel;
 mean_dist_travel=mean(dist_travel1);
 pop2=squeeze(pop1(1,:,:));
else
    dist_travel1=0;
    pop2=squeeze(pop1(1,:,:));
end
%% Plotting the final chromosome configuration with obstacles
        figure(1)
        set (gcf, 'color', 'w');
        hold off
        plot(squeeze(pop1((1),:,1)),squeeze(pop1((1),:,2)),'.r','MarkerSize',8);
        hold on;
        plot(path(:,1),path(:,2),'-b');
        hold on
        plot([obstacles{3}(:,1);obstacles{3}(1,1)] ,[obstacles{3}(:,2);obstacles{3}(1,2)],'-g');
        hold on;
        plot(15,15,'.c','MarkerSize',80);
        hold on
        plot(35,40,'.c','MarkerSize',80);
        axis([0 axis_scale 0 axis_scale]);
        title(['Generation = ' num2str(gen) ', Min Fitness = ' sprintf('%0.3f', minFitnessHist(1,gen+1))]);
        ylabel('Y points');
        xlabel('X points');
        drawnow;

end
%% plotting the minimum fitness values at one waypoint
    mean_dist_travel
    figure(2)
    plot([0:maxGens],minFitnessHist,'k-');
    title('Minimum Fitness for one waypoint')
    xlabel('Generation')
    ylabel('Fitness')

   

%% Calculating the fitness values for a generation
function fitness_val=fitnessfunc(pop,goal,obstacles)

for config_iter=1:size(pop(:,1))
    
pop_new=squeeze(pop(config_iter,:,:));
%% function for distance from goal metric
sum_dg=distance_goal(pop_new,goal);

%% function for nearest robot
sum_nearest=finding_nearestdist(pop_new);
sum_r=finding_repulsive(obstacles,pop_new);

fitness_val(config_iter)=sum_dg+sum_nearest+sum_r;
end

end
%% function for distance from goal metric
function sum_dg=distance_goal(pop_new,goal)
sum_dg=0;
for iter=1:size(pop_new,1)
     x=pop_new(iter,1);
     y=pop_new(iter,2);   
   sum_dg=sum_dg+sqrt((goal(1)-x)^2+(goal(2)-y)^2);
end
end
%% function for nearest robot
function sum_nearest=finding_nearestdist(pop_new)
sum_nearest=0;
L=0.5;
K=1.5;
 for iter=1:size(pop_new,1)
     x=pop_new(iter,1);
     y=pop_new(iter,2);  
     min_neigh=10^8;
     pop_new2=pop_new;
     pop_new2(iter,:)=[];
   for bots_iter=1:size(pop_new2,1)     
       x2=pop_new2(bots_iter,1);
       y2=pop_new2(bots_iter,2);
       dist_bot=sqrt((x2-x)^2+(y2-y)^2);  
       
       if(dist_bot<min_neigh)
           min_neigh=dist_bot;
           pos=bots_iter;
       end
       
   end
   chi_sq=(L-min_neigh)^2;
   if(min_neigh<=L)
       sum_nearest=10^8;
   else
       sum_nearest=sum_nearest+(0.5*K*chi_sq);
   end
 end
end
%% function for finding repulsive function 
function sum_r=finding_repulsive(obstacles,pop_new)
rho=0.4; 
Q=2; % safest distance from the obstacle
eta=0.8; % repulsive constant
sum_r=0;
obstacles1=[];
for j=1:length(obstacles)
    obstacles1=[obstacles1;obstacles{j}];
end
for iter =1:size(pop_new,1)
    x=pop_new(iter,1);
    y=pop_new(iter,2);  
   for j=1:length(obstacles1)
         dis(j)=sqrt((obstacles1(j,1)-x)^2+(obstacles1(j,2)-y)^2);   
   end
   [min_val,pos]=min(dis);
   if(min_val>Q)    
       sum_r=sum_r+((1/2)*(eta)*(((1/min_val)-(1/Q))^2));
   else
       sum_r=10^8;
   end 
end
sum_r=sum_r*rho;
end
end