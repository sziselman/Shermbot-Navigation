#include "rigid2d.hpp"
#include "rigid2d.cpp"
#include <iostream>


int main(void)
{
    using namespace std;

    Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
    cout << "Please enter transform T_ab:" << endl;
    cin >> T_ab;

    cout << "Please enter transform T_bc:" << endl;
    cin >> T_bc;

    T_ba = T_ab.inv();
    T_cb = T_bc.inv();

    T_ac = T_ab*T_bc;
    T_ca = T_ac.inv();
    
    cout << T_ab << endl;
    cout << T_ba << endl;
    cout << T_bc << endl;
    cout << T_cb << endl;
    cout << T_ac << endl;
    cout << T_ca << endl;
    return 0;
}