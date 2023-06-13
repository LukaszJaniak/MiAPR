# MiAPR
W ramach projektu z przedmiotu metody i algorytmy planowania ruchu realizowano implementację algorytmu PRM* jako planera ścieżki ruchu dla robota mobilnego Turtlebot3 w Navigation stack bazując na środowisku ROS 2 Humble. Całość zrealizowano w formie pluginu 
Implementacji planera ruchu dokonano w języku C++ bazując na instrukcji:

https://navigation.ros.org/getting_started/index.html#running-the-example

oraz

https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html

dokonano niezbędnych implemnetacji w celu realizacji projektu.


### Pseudokod,na podstwie którego realizowano implementację kodu PRM:

G(V,E) = Null //Initialize a graph as empty <br />
limit = n //number of nodes to make graph out of <br />
Rad = r //radius of neighborhoods <br />
For itr in 0...limit: <br />
    Xnew = RandomPosition() <br />
    Xnearest = Near(G(V,E),Xnew,Rad) //find all nodes within a Rad <br />
    Xnearest = sort(Xnearest) //sort by increasing distance <br />
    For node in Xnearest: <br />
        if not ConnectedComp(Xnew,node) and not Obstacle(Xnew,node): <br />
            G(V,E) += {Xnew,node} //add edge and node to graph <br />
            Xnew.comp += node.comp //add Xnew to connected component <br />
Return G(V,E)
(https://theclassytim.medium.com/robotic-path-planning-prm-prm-b4c64b1f5acb)

Następnie modyfikowano w.w. implementację do algorytmu PRM* poprzez wykorzystanie dynamicznego promienia, zależnego od ilości istniejących punktów, w którym łączono wierzchołki.

Planowania dokonywane jest w każdej pętli programu poprzez wykorzystamoe do tego sprogowanej lokalnej mapy kosztów, co umożliwia reagowanie na zmiany mapy wykryte przez sensor robota. 

### Schemat uruchomienia algorytmu:

1. git clone https://github.com/LukaszJaniak/MiAPR
2. edycja pliku /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml na zawarty w repozytorium nav2_params.yaml 
3. cd MiAPR/
4. ./prepare_env.sh
7. ./run_planner.sh
