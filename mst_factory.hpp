#ifndef MST_FACTORY_HPP
#define MST_FACTORY_HPP

#include "mst_strategy.hpp"
#include "prim_mst.hpp"
#include "kruskal_mst.hpp"
#include <memory>

enum MSTType {
    PRIM,
    KRUSKAL
};

class MSTFactory {
public:
    static std::unique_ptr<MSTStrategy> createMST(MSTType type);
};

#endif // MST_FACTORY_HPP
