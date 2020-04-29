//code is compiled using g++ -g fileName.cpp -o fileName.exe -std=c++2a
#include <bits/stdc++.h>
#include "load_flow.hpp"
using namespace std;

int main()
{
    // cook your dish here
    load_flow obj("busInputData.csv", "lineInputData.csv", 50);

    obj.printBusData();
    cout << "\n";

    obj.printLineData();
    cout << "\n";

    obj.calculateYBus();
    cout << "\n";

    obj.printYbus();
    cout << "\n";

    obj.solveLoadFlow();
    cout << "\n";

    obj.printBusData();
    cout << "\n";
    return 0;
}