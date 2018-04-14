
#include "Graduation_Trakya/graph.h"

//Construct the arena graph:

void load_graph (vertex_set *graph){
    
    //VERTEX 0(station0):
    graph[0].id = 0;            
    graph[0].x = -10;           //x coordinate
    graph[0].y = 10;           //y coordinate
    graph[0].z = 1;           //z coordinate
    graph[0].visits = 0;        //not visited yet.
    
    graph[0].num_neigh = 1;     //vertex 0 has 1 neighbor
    graph[0].id_neigh[0] = 7;   //the neighbor of vertex 0 is vertex 1 it's where the first drone will start patrolling from


    //VERTEX 1(station1):
    graph[1].id = 1;            
    graph[1].x = 10;           //x coordinate
    graph[1].y = 10;           //y coordinate
    graph[1].z = 2;           //z coordinate
    graph[1].visits = 0;        //not visited yet.
    
    graph[1].num_neigh = 1;     //vertex 0 has 1 neighbor
    graph[1].id_neigh[0] = 14;   //the neighbor of vertex 0 is vertex 1 it's where the second drone will start patrolling from

    //VERTEX 2(station2):
    graph[2].id = 2;            
    graph[2].x = 0;           //x coordinate
    graph[2].y = 10;           //y coordinate
    graph[2].z = 3;           //z coordinate
    graph[2].visits = 0;        //not visited yet.
    
    graph[2].num_neigh = 1;     //vertex 0 has 1 neighbor
    graph[2].id_neigh[0] = 6;   //the neighbor of vertex 0 is vertex 1 it's where the third drone will start patrolling from


    //VERTEX 3:
    graph[3].id = 3;            
    graph[3].x = 0.9;           //x coordinate
    graph[3].y = 0.9;           //y coordinate
    graph[3].z = 0.5;           //z coordinate
    graph[3].visits = 0;        //not visited yet.
    
    graph[3].num_neigh = 1;     //vertex 3 has 1 neighbor
    graph[3].id_neigh[0] = 4;   //the neighbor of vertex 3 is vertex 4
    
    
    //VERTEX 4:
    graph[4].id = 4;            
    graph[4].x = 3.0;           //x coordinate
    graph[4].y = 0.9;           //y coordinate
    graph[4].z = 0.5;           //z coordinate
    graph[4].visits = 0;        //not visited yet.
    
    graph[4].num_neigh = 3;     //vertex 4 has 3 neighbors
    
    graph[4].id_neigh[0] = 3;   //the 1st neighbor of vertex 4 is vertex 3
 //0.    
    
    graph[4].id_neigh[1] = 5;   //the 2nd neighbor of vertex 4 is vertex 5
 //2.    
    
    graph[4].id_neigh[2] = 8;   //the 3rd neighbor of vertex 4 is vertex 8

    
    
    //VERTEX 5:
    graph[5].id = 5;            
    graph[5].x = 5.1;           //x coordinate
    graph[5].y = 0.9;           //y coordinate
    graph[5].z = 0.5;           //z coordinate
    graph[5].visits = 0;        //not visited yet.
    
    graph[5].num_neigh = 3;     //vertex 5 has 3 neighbors
    
    graph[5].id_neigh[0] = 4;   //the 1st neighbor of vertex 5 is vertex 4
 //1.    
    
    graph[5].id_neigh[1] = 6;   //the 2nd neighbor of vertex 5 is vertex 6
 //3.    
    
    graph[5].id_neigh[2] = 9;   //the 3rd neighbor of vertex 5 is vertex 9
 //6.   
    
    
    //VERTEX 6:
    graph[6].id = 6;            
    graph[6].x = 6.9;           //x coordinate
    graph[6].y = 0.9;           //y coordinate
    graph[6].z = 0.5;           //z coordinate
    graph[6].visits = 0;        //not visited yet.
    
    graph[6].num_neigh = 2;     //vertex 6 has 2 neighbors
    
    graph[6].id_neigh[0] = 5;   //the 1st neighbor of vertex 6 is vertex 5
 //2.    
    
    graph[6].id_neigh[1] = 10;   //the 2nd neighbor of vertex 6 is vertex 10
 //7.       
    
    
    //VERTEX 7:
    graph[7].id = 7;            
    graph[7].x = 0.9;           //x coordinate
    graph[7].y = 2.7;           //y coordinate
    graph[7].z = 0.5;           //z coordinate
    graph[7].visits = 0;        //not visited yet.
    
    graph[7].num_neigh = 2;     //vertex 7 has 2 neighbors
    
    graph[7].id_neigh[0] = 8;   //the 1st neighbor of vertex 7 is vertex 8
 //5.    
    
    graph[7].id_neigh[1] = 11;   //the 2nd neighbor of vertex 7 is vertex 11
 //8.           
     
    
    //VERTEX 8:
    graph[8].id = 8;            
    graph[8].x = 2.8;           //x coordinate
    graph[8].y = 2.7;           //y coordinate
    graph[8].z = 0.5;           //z coordinate
    graph[8].visits = 0;        //not visited yet.
    
    graph[8].num_neigh = 3;     //vertex 8 has 3 neighbors
    
    graph[8].id_neigh[0] = 4;   //the 1st neighbor of vertex 8 is vertex 4
 //1.    
    
    graph[8].id_neigh[1] = 7;   //the 2nd neighbor of vertex 8 is vertex 7
 //4.    
    
    graph[8].id_neigh[2] = 12;   //the 3rd neighbor of vertex 8 is vertex 12
 //9.    

      
    //VERTEX 9:
    graph[9].id = 9;            
    graph[9].x = 4.7;           //x coordinate
    graph[9].y = 2.7;           //y coordinate
    graph[9].z = 0.5;           //z coordinate
    graph[9].visits = 0;        //not visited yet.
    
    graph[9].num_neigh = 2;     //vertex 9 has 2 neighbors
    
    graph[9].id_neigh[0] = 5;   //the 1st neighbor of vertex 9 is vertex 5
 //2.    
    
    graph[9].id_neigh[1] = 10;   //the 2nd neighbor of vertex 9 is vertex 10


      
    //VERTEX 10:
    graph[10].id = 10;            
    graph[10].x = 6.9;           //x coordinate
    graph[10].y = 2.7;           //y coordinate
    graph[10].z = 0.5;           //z coordinate
    graph[10].visits = 0;        //not visited yet.
    
    graph[10].num_neigh = 2;     //vertex 10 has 2 neighbors
    
    graph[10].id_neigh[0] = 6;   //the 1st neighbor of vertex 10 is vertex 6
 //3.    
    
    graph[10].id_neigh[1] = 9;   //the 2nd neighbor of vertex 10 is vertex 9
 //6.      

      
    //VERTEX 11:
    graph[11].id = 11;            
    graph[11].x = 1.2;           //x coordinate
    graph[11].y = 4.5;           //y coordinate
    graph[11].z = 0.5;           //z coordinate
    graph[11].visits = 0;        //not visited yet.
    
    graph[11].num_neigh = 2;     //vertex 11 has 2 neighbors
    
    graph[11].id_neigh[0] = 7;   //the 1st neighbor of vertex 11 is vertex 7
 //4.    
    
    graph[11].id_neigh[1] = 12;   //the 2nd neighbor of vertex 11 is vertex 12


      
    //VERTEX 12:
    graph[12].id = 12;            
    graph[12].x = 3.3;           //x coordinate
    graph[12].y = 4.5;           //y coordinate
    graph[12].z = 0.5;           //z coordinate
    graph[12].visits = 0;        //not visited yet.
    
    graph[12].num_neigh = 3;     //vertex 9 has 3 neighbors
    
    graph[12].id_neigh[0] = 8;   //the 1st neighbor of vertex 12 is vertex 8
 //5.    
    
    graph[12].id_neigh[1] = 11;   //the 2nd neighbor of vertex 12 is vertex 11
 //8.     
    
    graph[12].id_neigh[2] = 13;  //the 3rd neighbor of vertex 12 is vertex 13

    
      
    //VERTEX 13:
    graph[13].id = 13;            
    graph[13].x = 5.1;           //x coordinate
    graph[13].y = 4.5;           //y coordinate
    graph[13].z = 0.5;           //z coordinate
    graph[13].visits = 0;        //not visited yet.
    
    graph[13].num_neigh = 2;     //vertex 13 has 2 neighbors
    
    graph[13].id_neigh[0] = 12;   //the 1st neighbor of vertex 13 is vertex 12
 //9.    
    
    graph[13].id_neigh[1] = 14;  //the 2nd neighbor of vertex 13 is vertex 14
 //11.      
    
    
      
    //VERTEX 14:
    graph[14].id = 14;            
    graph[14].x = 6.9;           //x coordinate
    graph[14].y = 4.5;           //y coordinate
    graph[14].z = 0.5;           //z coordinate
    graph[14].visits = 0;        //not visited yet.
    
    graph[14].num_neigh = 1;     //vertex 14 has 1 neighbor
    
    graph[14].id_neigh[0] = 13;  //the 1st neighbor of vertex 14 is vertex 13
 //10.    
}
