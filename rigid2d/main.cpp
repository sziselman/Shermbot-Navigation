#include "rigid2d.hpp"
#include <iostream>
#include <string>


int main(void)
{
    using namespace std;

    rigid2d::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
    rigid2d::Vector2D vector;
    string frame;

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

    cout << "Please enter a 2D vector:" << endl;
    cin >> vector;
    cout << "Please enter a frame in which it is defined (either 'a', 'b', or 'c'):" << endl;
    cin >> frame;

    cout << "You entered the 2D vector:" << vector << endl;
    cout << "You entered the frame:" << frame << endl;

    return 0;
}