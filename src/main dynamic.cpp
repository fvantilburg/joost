#include <cstdlib>
#include <iostream>
#include "dlib/matrix.h"
#include "dlib/matrix/matrix_abstract.h"
#include <complex>

using namespace std;
using namespace dlib;

matrix<double,2,1> tau;

double t1 = 10,
       t2 = 0,
       w1 = 0,
       w2 = 0,
       a1 = 0,
       a2 = 0;

double const L1   = 0.3475,
             L2   = 0.3333,
             lc1  = 0.1737,
             lc2  = 0.1666,
             Izz1 = 0.0001,
             Izz2 = 0.0001,  
             m1   = 0.9000,
             m2   = 0.9000,
             g    = 9.8100;

// initArm function
void EoM()  {

    for (int i = 0; i <= 90; i++) {
    
    t1 = i * pi/180;
    //t2 = i * 3.1415/180;
    
    tau = -L1*lc2*m2*sin(t2)*pow(w2,2) - 2*L1*lc2*m2*w1*sin(t2)*w2 + a2*(Izz2 + m2*(pow(lc2,2) + L1*cos(t2)*lc2)) + a1*(m1*pow(lc1,2) + Izz1 + Izz2 +
            m2*(pow(L1,2) + 2*cos(t2)*L1*lc2 + pow(lc2,2))) + g*m2*(lc2*cos(t1 + t2) + L1*cos(t1)) + g*lc1*m1*cos(t1),
            L1*lc2*m2*sin(t2)*pow(w1,2) + a2*(m2*pow(lc2,2) + Izz2) + a1*(Izz2 + m2*(pow(lc2,2) + L1*cos(t2)*lc2)) + g*lc2*m2*cos(t1 + t2);

      cout << "t1 = " << t1 << endl << "tau=\n" << tau << endl;
    }

}



int main(int argc, char *argv[])
{
    EoM();
    
    system("PAUSE");
    cout << "Press any letter or symbol key and then enter to stop" << endl;
    cin >> t1;
    return EXIT_SUCCESS;
}
