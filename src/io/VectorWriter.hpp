
#include "../datastructures.hpp"
#include "../projHelper.hpp"
#include <cstddef>
#include <memory>

namespace roofer {
  struct VectorWriterInterface {

    projHelperInterface& pjHelper;

    VectorWriterInterface(projHelperInterface& pjh) : pjHelper(pjh) {};

    virtual void writePolygons(
      const std::string& source, 
      const std::vector<LinearRing>& polygons, 
      const AttributeVecMap& attributes,
      size_t begin,
      size_t end) = 0;

    void writePolygons(
      const std::string& source, 
      const std::vector<LinearRing>& polygons, 
      const AttributeVecMap& attributes) {
      writePolygons(source, polygons, attributes, 0, polygons.size());
    };
  };

  std::unique_ptr<VectorWriterInterface> createVectorWriterOGR(projHelperInterface& pjh);
}