#include "mst_factory.hpp"

std::unique_ptr<MSTStrategy> MSTFactory::createMST(MSTType type) {
    if (type == PRIM) {
        return std::make_unique<PrimMST>();
    } else if (type == KRUSKAL) {
        return std::make_unique<KruskalMST>();
    }
    return nullptr;
}
