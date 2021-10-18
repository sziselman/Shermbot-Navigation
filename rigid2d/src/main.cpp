#include "rigid2d.hpp"
#include <iostream>
#include <string>

int main(void)
{
    using namespace std;

    rigid2d::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
    rigid2d::Vector2D vector, vector_a, vector_b, vector_c;
    rigid2d::Twist2D twist, twist_a, twist_b, twist_c;
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

    while ((frame != "a") && (frame != "b") && (frame != "c"))
    {
        cout << "Please enter a frame in which it is defined (either a, b or c):" << endl;
        cin >> frame;
    }

    if (frame == "a")
    {
        vector_a = vector;
        vector_b = T_ba(vector_a);
        vector_c = T_ca(vector_a);
        cout << "The vector in the a frame is: " << vector_a << endl;
        cout << "The vector in the b frame is: " << vector_b << endl;
        cout << "The vector in the c frame is: " << vector_c << endl;
    }
    else if (frame == "b")
    {
        vector_b = vector;
        vector_a = T_ab(vector_b);
        vector_c = T_cb(vector_b);
        cout << "The vector in the a frame is: " << vector_a << endl;
        cout << "The vector in the b frame is: " << vector_b << endl;
        cout << "The vector in the c frame is: " << vector_c << endl;
    }
    else if (frame == "c")
    {
        vector_c = vector;
        vector_a = T_ac(vector_c);
        vector_b = T_bc(vector_c);
        cout << "The vector in the a frame is: " << vector_a << endl;
        cout << "The vector in the b frame is: " << vector_b << endl;
        cout << "The vector in the c frame is: " << vector_c << endl;
    }

    cout << "Please enter a 2D twist:" << endl;
    cin >> twist;

    if (frame == "a")
    {
        twist_a = twist;
        twist_b = T_ba(twist_a);
        twist_c = T_ca(twist_a);
        cout << "The twist in the a frame is: " << twist_a << endl;
        cout << "The twist in the b frame is: " << twist_b << endl;
        cout << "The twist in the c frame is: " << twist_c << endl;
    }
    else if (frame == "b")
    {
        twist_b = twist;
        twist_a = T_ab(twist_b);
        twist_c = T_cb(twist_b);
        cout << "The twist in the a frame is: " << twist_a << endl;
        cout << "The twist in the b frame is: " << twist_b << endl;
        cout << "The twist in the c frame is: " << twist_c << endl;
    }
    else if (frame == "c")
    {
        twist_c = twist;
        twist_a = T_ac(twist_c);
        twist_b = T_bc(twist_c);
        cout << "The twist in the a frame is: " << twist_a << endl;
        cout << "The twist in the b frame is: " << twist_b << endl;
        cout << "The twist in the c frame is: " << twist_c << endl;
    }
    return 0;
}