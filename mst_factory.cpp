#include "mst_factory.hpp"
#include <stdexcept>

std::unique_ptr<MSTStrategy> MSTFactory::createMST(MSTType type) {
    switch (type) {
        case MSTType::PRIM:
            return std::make_unique<PrimMST>();
        case MSTType::KRUSKAL:
            return std::make_unique<KruskalMST>();
        default:
            throw std::runtime_error("Unknown MST type");
    }
}
