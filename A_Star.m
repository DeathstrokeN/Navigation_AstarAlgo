clf;
%Définie la carte 2D de l'environnement (cellules)
MAX_X=10;   % nombre maximal de cellule selon x
MAX_Y=10;% nombre maximal de cellule selon y
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

% Introduire les positions de départ, but et obstacles
% le codage des données dans Map sera :  Obstacle=-1, But = 0, Départ=1, Vide=2
j=0;
x_val = 1;
y_val = 1;
axis([1 MAX_X+1 1 MAX_Y+1])
grid on;
hold on;
n=0;%Nombre d'obstacles

% BEGIN introduire d'une manière intéractive les positions: Obstacle, But, Départ
pause(1);
h=msgbox('Sélectionner la Position de But (Target) utilisant le bouton gauche de la souris');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('Sélectionner la Position de But utilisant le bouton gauche de la souris','Color','black');
but=0;
while (but ~= 1) %Refaire tant que le bouton n'est pas clické
    [xval,yval,but]=ginput(1);
end
xval=floor(xval);
yval=floor(yval);
xTarget=xval;%X coordonnée de Target
yTarget=yval;%Y  coordonnée de Target

MAP(xval,yval)=0;%Initializer MAP avec la position de but (target)
plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'Target')

pause(2);
h=msgbox('Sélectionner les obstacles utilisant le BG de la souris, indiquer le dernier obstacle par le BD de la souris');
  xlabel('Sélectionner les obstacles utilisant le BG de la souris, indiquer le dernier obstacle par le BD de la souris','Color','blue');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=-1;%introduire les données dans MAP
    plot(xval+.5,yval+.5,'ro');
 end%End de la boucle While 
 
pause(1);

h=msgbox('Sélectionner la Position de départ (Start) utilisant le bouton gauche de la souris');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('Sélectionner la Position de départ (Start) utilisant le bouton gauche de la souris','Color','black');
but=0;
while (but ~= 1) %Refaire tant que le bouton n'est pas clické
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
end
xStart=xval;% x coordonnée de la position de départ
yStart=yval;% y coordonnée de la position de départ
MAP(xval,yval)=1;
 plot(xval+.5,yval+.5,'bo');
%Fin de la création de la carte et le choix des positions de départ et de %fin

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS utilisée dans l'ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%la liste OPEN 
%--------------------------------------------------------------------------
%chaque ligne de OPEN contient 8 éléments:  1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
% La liste CLOSED

%--------------
%chaque ligne contient 2 éléments : X val | Y val |
%--------------

CLOSED=[];

%mettre tous les obstacles dans CLOSE
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%le premier noeud est le noeud de départ Start
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
% calcul de la distance entre la position de Start et Target
goal_distance=distance(xNode,yNode,xTarget,yTarget);
%introduire cette mesure dans la liste OPEN
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
% Insérer votre code ici ( pathcost cout h)
    expanding_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y); 
    %cpt:compteur
    expanding_cpt=size(expanding_array,1);
    
    for i=1:expanding_cpt
        flag=0;
        for j=1;OPEN_COUNT
            if(expanding_array(i,1)== OPEN(j,2) && expanding_array(i,2) == OPEN(j,3))
                OPEN(j,8)=min(OPEN(j,8),expanding_array(i,5));
                if OPEN(j,8) == expanding_array(i,5)
                    OPEN(j,4)=xNode;
                    OPEN(j,5)=yNode;
                    OPEN(j,6)=expanding_array(i,3);
                    OPEN(j,7)=expanding_array(i,3);
                end
                flag=1;
            end
        end
        if flag==0
            OPEN_COUNT=OPEN_COUNT+1;
            OPEN(OPEN_COUNT,:)=insert_open(expanding_array(i,1),expanding_array(i,2),xNode,yNode,expanding_array(i,3),expanding_array(1,4),expanding_array(1,5))
        end
    end
    
    %maintenant trouvons min(f(n))
    
    MinNode=min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
    if (MinNode~=-1)
        xNode=OPEN(MinNode,2);
        yNode=OPEN(MinNode,3);
        path_cost= OPEN(MinNode,6);
        %ajout du node a CLOSED
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=xNode;
        CLOSED(CLOSED_COUNT,2)=yNode;
        OPEN(MinNode,1)=0;
    else %si on trouve pas de noeud
        NoPath=0;
        %B=msgbox('stop le programme "pas de chemin possible"');
       % break;
    end

end;
 


% on récupère le chemin optimal en démarrant de la position de Target
% jusqu'à la position de Start en suivant le noeud parent à chaque fois

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;

if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverser OPEN et déterminer les noeuds parents
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index retourne l(indice d'un noeud
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %récupérer les grands parents
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index retourne l(indice d'un noeud
           parent_y=OPEN(inode,5);
           i=i+1;
    end;
 j=size(Optimal_path,1);
 %Plot le chemin optimal
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 for i=j:-1:1
  pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end;
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, Pas de chemin menant à Target!','warn');
 uiwait(h,5);
end

    





