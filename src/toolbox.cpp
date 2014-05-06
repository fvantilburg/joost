#include "toolbox.h"



//globals

matrix<double,4,4> getT(int a, int b){
    matrix<double,4,4> Rt, Pd, Ra, Pa;
    
    // identity if frames are identical
    if (a==b) return identity_matrix<double>(4);
    
    //forward begin and end
    if (a==0) return T01*getT(a+1,b);
    if (b==Number_of_joints+1) return getT(a,b-1)*Tend;
    
    //backward begin and end
    if (b==0) return getT(a,b+1)*inv(T01);
    if (a==Number_of_joints+1) return inv(Tend)*getT(a-1,b);
    
    //transforms in between are calclated via DH parameters
    if (a<b){
        // translation along X    
        Pa= 1, 0, 0, DH(b-1,1),
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
         // rotation of Z about X
        Ra= 1, 0, 0, 0, 
            0, cos(DH(b-1,0)), -sin(DH(b-1,0)), 0,
            0, sin(DH(b-1,0)),  cos(DH(b-1,0)), 0,
            0, 0, 0, 1;
        // translation along Z
        Pd= 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, DH(b-1,2),
            0, 0, 0, 1;
        //rotation of X about Z
        Rt= cos(DH(b-1,3)), -sin(DH(b-1,3)), 0, 0,
            sin(DH(b-1,3)),  cos(DH(b-1,3)), 0, 0,
            0 ,0 ,1 ,0,
            0, 0, 0, 1;
            
        return getT(a,b-1)*Pa*Ra*Pd*Rt;
    } else {
        
        //rotation of X about Z
        Rt= cos(-DH(a-1,3)), -sin(-DH(a-1,3)), 0, 0,
            sin(-DH(a-1,3)),  cos(-DH(a-1,3)), 0, 0,
            0 ,0 ,1 ,0,
            0, 0, 0, 1;
        // translation along Z
        Pd= 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, -DH(a-1,2),
            0, 0, 0, 1;
        // rotation of Z about X
        Ra= 1, 0, 0, 0, 
            0, cos(-DH(a-1,0)), -sin(-DH(a-1,0)), 0,
            0, sin(-DH(a-1,0)),  cos(-DH(a-1,0)), 0,
            0, 0, 0, 1;
        // translation along X    
        Pa= 1, 0, 0, -DH(a-1,1),
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return Rt*Pd*Ra*Pa*getT(a-1,b); 
    }    
}
 
matrix<double,3,Number_of_joints+2> getP(){
    // returns x,y,z coordinates of all frames 
    matrix<double,4,Number_of_joints+2> P;
    
     //select position column from identity matrix
    set_colm(P,0)= colm(identity_matrix<double>(4),3);

    for (int i = 1 ; i<Number_of_joints+2 ;i++){  
        set_colm(P,i) = getT(i-1,i)*colm(P,i-1);
    }
    return rowm(P,range(0,2));    
}

matrix<double,3,1> getP(int frame){
    // returns x,y,z coordinates of selected frame
    return subm(getT(0,frame),range(0,2),range(3,3));    
}

matrix<double,3,Number_of_joints+2> getZ(){
    // returns coordinates of the z axes of each frame expressed in frame 0
    matrix<double,4,Number_of_joints+2> Z;
    
    //select Z axis from identity matrix
    set_colm(Z,0)= colm(identity_matrix<double>(4),2);


    for (int i = 1 ; i<Number_of_joints+2 ;i++){    
        set_colm(Z,i) = getT(i-1,i)*colm(Z,i-1);
    }
    return rowm(Z,range(0,2));    
}

matrix<double,6,Number_of_joints> getJ(){
    // returns the jacobian of the arm
    matrix<double,6,Number_of_joints> J;
    matrix<double,3,Number_of_joints+2> P;
    matrix<double,3,Number_of_joints+2> Z;
    
    // P and Z are calculated. Room for improvement because T sequence is calculated twice.
    P = getP();
    Z = getZ();

    // J is calculated    
    for (int i = 0;i<Number_of_joints;i++){
        if (E(i)==1) { //prismatic joint
            set_subm(J,range(0,2),range(i,i))=colm(Z,i+1);
            set_subm(J,range(3,5),range(i,i))=0;
        }
        else{         // rotational joint
            set_subm(J,range(0,2),range(i,i))=cross(colm(P,Number_of_joints+1)-colm(P,i+1),colm(Z,i+1));
            set_subm(J,range(3,5),range(i,i))=colm(Z,i+1);
        }    
    }  
    return J; 
}
 
 
matrix<double,3,1> cross(matrix<double,3,1> A , matrix<double,3,1> B){
    // cross product implementation for matrixes
    matrix<double,3,1> C;
    C(0)=A(1)*B(2)-A(2)*B(1);
    C(1)=A(2)*B(0)-A(0)*B(2);
    C(2)=A(0)*B(1)-A(1)*B(0);
    return C;  
}
