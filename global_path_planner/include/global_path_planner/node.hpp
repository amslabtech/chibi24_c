#ifndef NODE
#define NODE

#include <cmath>
#include <memory>

struct ANode
{
    int x = 0;
    int y = 0;
    double g = 0;                            // スタートからの実コスト
    double h = 0;                            // ヒューリスティックコスト
    double cost = 0;                         // コスト
    std::shared_ptr<ANode> parent = nullptr; // 親ノードへのポインタ

    ANode(int x, int y, double g, std::shared_ptr<ANode> goal, std::shared_ptr<ANode> parent = nullptr)
        : x(x),
          y(y),
          g(g),
          parent(parent)
    {
        h = (goal) ? std::hypot(goal->x - x, goal->y - y) : 0;
    }

  
    ANode(int x, int y, double g)
        : ANode(x, y, g, nullptr, nullptr) {}

    double getF() const
    {
        return g + h;
    }

    bool operator<(const ANode &node) const
    {
        return getF() < node.getF();
    }

    bool operator>(const ANode &node) const
    {
        return getF() > node.getF();
    }

    bool operator==(const ANode &node) const
    {
        return x == node.x && y == node.y;
    }
};


#endif