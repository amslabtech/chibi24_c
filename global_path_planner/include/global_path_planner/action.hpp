#ifndef ACTION
#define ACTION

struct Action
{
    int x = 0;
    int y = 0;
    double cost = 0;

    Action(int px, int py, double pCost) : x(px), y(py), cost(pCost) {}
};


#endif