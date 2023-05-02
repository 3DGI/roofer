// This file is part of gfp-gdal
// Copyright (C) 2018-2022 Ravi Peters

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "VectorReader.hpp"

#include <unordered_map>
#include <variant>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <filesystem>

#include <ogrsf_frmts.h>

namespace fs = std::filesystem;

namespace roofer {

// void OGRLoaderNode::push_attributes(const OGRFeature &poFeature, std::unordered_map<std::string,int>& fieldNameMap)
// {
//   for (auto &[name, mterm] : poly_output("attributes").sub_terminals())
//   {
//     if (mterm->accepts_type(typeid(bool)))
//     {
//       mterm->push_back(bool(poFeature.GetFieldAsInteger(name.c_str())));
//     }
//     else if (mterm->accepts_type(typeid(int)))
//     {
//       if(name == "OGR_FID") {
//         mterm->push_back(int(poFeature.GetFID()));
//       } else {
//         mterm->push_back(int(poFeature.GetFieldAsInteger64(name.c_str())));
//       }
//     }
//     else if (mterm->accepts_type(typeid(float)))
//     {
//       mterm->push_back(float(poFeature.GetFieldAsDouble(name.c_str())));
//     }
//     else if (mterm->accepts_type(typeid(std::string)))
//     {
//       mterm->push_back((std::string)poFeature.GetFieldAsString(name.c_str()));
//     }
//     else if (mterm->accepts_type(typeid(Date)))
//     {
//       DateTime t;
//       poFeature.GetFieldAsDateTime(fieldNameMap[name], &t.date.year, &t.date.month, &t.date.day, nullptr, nullptr, &t.time.second, nullptr);
//       mterm->push_back(t.date);
//     }
//     else if (mterm->accepts_type(typeid(Time)))
//     {
//       Time time;
//       poFeature.GetFieldAsDateTime(fieldNameMap[name], nullptr, nullptr, nullptr, &time.hour, &time.minute, &time.second, &time.timeZone);
//       mterm->push_back(time);
//     }
//     else if (mterm->accepts_type(typeid(DateTime)))
//     {
//       DateTime t;
//       poFeature.GetFieldAsDateTime(fieldNameMap[name], &t.date.year, &t.date.month, &t.date.day, &t.time.hour, &t.time.minute, &t.time.second, &t.time.timeZone);
//       mterm->push_back(t);
//     }
//   }
// }

class VectorReaderOGR : public VectorReaderInterface {
  GDALDatasetUniquePtr poDS;

  int layer_count = 0;
  int layer_id = 0;
  std::string layer_name_ = "";
  std::string attribute_filter_ = "";
  float base_elevation = 0;
  bool output_fid_ = false;

  public:
  using VectorReaderInterface::VectorReaderInterface;
  
  void open(const std::string& source) {
    GDALAllRegister();
    poDS = GDALDatasetUniquePtr(GDALDataset::Open(source.c_str(), GDAL_OF_VECTOR));
    if (poDS == nullptr)
      throw(rooferException("Open failed on " + source));
  }

  void read_polygon(OGRPolygon* poPolygon, std::vector<LinearRing>& polygons) {
    LinearRing gf_polygon;
    // for(auto& poPoint : poPolygon->getExteriorRing()) {
    OGRPoint poPoint;
    auto ogr_ering = poPolygon->getExteriorRing();
    
    // ensure we output ccw exterior ring
    if ( ogr_ering->isClockwise() ) {
      ogr_ering->reverseWindingOrder();
    }
    for (size_t i = 0; i < ogr_ering->getNumPoints() - 1; ++i)
    {
      ogr_ering->getPoint(i, &poPoint);
      std::array<float, 3> p = pjHelper.coord_transform_fwd(
          poPoint.getX(),
          poPoint.getY(),
          base_elevation==0 ? poPoint.getZ() : base_elevation
        );
      gf_polygon.push_back(p);
    }
    // also read the interior rings (holes)
    for (size_t i = 0; i < poPolygon->getNumInteriorRings(); ++i) 
    {
      auto ogr_iring = poPolygon->getInteriorRing(i);
      // ensure we output cw interior ring
      if ( !ogr_iring->isClockwise() ) {
        ogr_iring->reverseWindingOrder();
      }
      vec3f gf_iring;
      for (size_t j = 0; j < ogr_iring->getNumPoints() - 1; ++j)
      {
        ogr_iring->getPoint(j, &poPoint);
        std::array<float, 3> p = pjHelper.coord_transform_fwd(
          poPoint.getX(),
          poPoint.getY(),
          base_elevation==0 ? poPoint.getZ() : base_elevation
        );
        gf_iring.push_back(p);
      }
      gf_polygon.interior_rings().push_back(gf_iring);
    }
    polygons.push_back(gf_polygon);
  }

  std::vector<LinearRing> readPolygons()
  {
    layer_count = poDS->GetLayerCount();
    std::cout << "Layer count: " << layer_count << "\n";
    OGRLayer *poLayer;
    
    poLayer = poDS->GetLayerByName( layer_name_.c_str() );
    if (poLayer == nullptr) {
      if (layer_id >= layer_count) {
        throw(rooferException("Illegal layer ID! Layer ID must be less than the layer count."));
      } else if (layer_id < 0) {
        throw(rooferException("Illegal layer ID! Layer ID cannot be negative."));
      }
      poLayer = poDS->GetLayer( layer_id) ;
      // throw(rooferException("Could not get the selected layer by name=" + layer_name));
    }
    if (poLayer == nullptr)
      throw(rooferException("Could not get the selected layer "));


    std::cout << "Layer '" << poLayer->GetName() << "' feature count: " << poLayer->GetFeatureCount() << "\n";
    auto geometry_type = poLayer->GetGeomType();
    auto geometry_type_name = OGRGeometryTypeToName(geometry_type);
    std::cout << "Layer geometry type: " << geometry_type_name << "\n";

    auto layer_def = poLayer->GetLayerDefn();
    auto field_count = layer_def->GetFieldCount();

    // Set up vertex data (and buffer(s)) and attribute pointers
    // LineStringCollection line_strings;
    // LinearRingCollection linear_rings;
    std::vector<LinearRing> polygons;
    // auto &linear_rings = vector_output("linear_rings");
    // auto &line_strings = vector_output("line_strings");
    
    // auto &is_valid = vector_output("is_valid");
    // auto &area = vector_output("area");

    std::unordered_map<std::string,int> fieldNameMap;
    // for (size_t i = 0; i < field_count; ++i)
    // {
    //   auto field_def = layer_def->GetFieldDefn(i);
    //   auto t = field_def->GetType();
    //   auto field_name = (std::string)field_def->GetNameRef();
    //   fieldNameMap[field_name] = i;
    //   if ((t == OFTInteger) && (field_def->GetSubType() == OFSTBoolean)) 
    //   {
    //     auto &term = poly_output("attributes").add_vector(field_name, typeid(bool));
    //   } 
    //   else if (t == OFTInteger || t == OFTInteger64)
    //   {
    //     auto &term = poly_output("attributes").add_vector(field_name, typeid(int));
    //     // term.set(vec1i());
    //   }
    //   else if (t == OFTString)
    //   {
    //     auto &term = poly_output("attributes").add_vector(field_name, typeid(std::string));
    //     // term.set(vec1s());
    //   }
    //   else if (t == OFTReal)
    //   {
    //     auto &term = poly_output("attributes").add_vector(field_name, typeid(float));
    //     // term.set(vec1f());
    //   }
    //   else if (t == OFTDate)
    //   {
    //     auto &term = poly_output("attributes").add_vector(field_name, typeid(Date));
    //     // term.set(vec1f());
    //   }
    //   else if (t == OFTTime)
    //   {
    //     auto &term = poly_output("attributes").add_vector(field_name, typeid(Time));
    //     // term.set(vec1f());
    //   }
    //   else if (t == OFTDateTime)
    //   {
    //     auto &term = poly_output("attributes").add_vector(field_name, typeid(DateTime));
    //     // term.set(vec1f());
    //   }
    // }

    // if(output_fid_)
    //   auto &ogrfid_term = poly_output("attributes").add_vector("OGR_FID", typeid(int));

    poLayer->ResetReading();

    
    // if ((poLayer->GetFeatureCount()) < feature_select || feature_select < 0)
    //   throw rooferException("Illegal feature_select value");

    char *pszWKT = NULL;
    OGRSpatialReference* layerSRS = poLayer->GetSpatialRef();
    layerSRS->exportToWkt( &pszWKT );
    // printf( "Layer SRS: \n %s\n", pszWKT );
    pjHelper.set_fwd_crs_transform(pszWKT);
    CPLFree(pszWKT);

    if (attribute_filter_.size()) {
      auto attribute_filter = attribute_filter_;
      auto error_code = poLayer->SetAttributeFilter(attribute_filter.c_str());
      if (OGRERR_NONE != error_code) {
        throw(rooferException("Invalid attribute filter: OGRErr="+std::to_string(error_code)+", filter="+attribute_filter));
      }
    }

    size_t fid{1};
    OGRFeature *poFeature;
    while( (poFeature = poLayer->GetNextFeature()) != NULL )
    // for (auto &poFeature : poLayer)
    {
      // if(feature_select != 0 && fid++ != feature_select) continue;

      // read feature geometry
      OGRGeometry *poGeometry;
      poGeometry = poFeature->GetGeometryRef();
      // std::cout << "Layer geometry type: " << poGeometry->getGeometryType() << " , " << geometry_type << "\n";
      if (poGeometry != nullptr) // FIXME: we should check if te layer geometrytype matches with this feature's geometry type. Messy because they can be a bit different eg. wkbLineStringZM and wkbLineString25D
      {

        // if (wkbFlatten(poGeometry->getGeometryType()) == wkbLineString)
        // {
        //   OGRLineString *poLineString = poGeometry->toLineString();

        //   LineString line_string;
        //   for (auto &poPoint : poLineString)
        //   {
        //     std::array<float, 3> p = pjHelper.coord_transform_fwd(
        //       poPoint.getX(),
        //       poPoint.getY(),
        //       base_elevation==0 ? poPoint.getZ() : base_elevation
        //     );
        //     line_string.push_back(p);
        //   }
        //   line_strings.push_back(line_string);
        //   is_valid.push_back(bool(poGeometry->IsValid()));

        //   // push_attributes(*poFeature, fieldNameMap);
        // }
        // else 
        if (wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon)
        {
          OGRPolygon *poPolygon = poGeometry->toPolygon();

          read_polygon(poPolygon, polygons);
          
          // area.push_back(float(poPolygon->get_Area()));
          // is_valid.push_back(bool(poPolygon->IsValid()));

          // push_attributes(*poFeature, fieldNameMap);

        } 
        else if ( wkbFlatten(poGeometry->getGeometryType()) == wkbMultiPolygon ) 
        {
          OGRMultiPolygon *poMultiPolygon = poGeometry->toMultiPolygon();
          for (auto poly_it = poMultiPolygon->begin(); poly_it != poMultiPolygon->end(); ++poly_it) {
            read_polygon(*poly_it, polygons);
          
            // area.push_back(float((*poly_it)->get_Area()));
            // is_valid.push_back(bool((*poly_it)->IsValid()));

            // push_attributes(*poFeature, fieldNameMap);
          }
        } else {
          throw rooferException("Unsupported geometry type\n");
        }
      }
    }
    // if (geometry_type == wkbLineString25D || geometry_type == wkbLineStringZM) {
    // if (line_strings.size() > 0)
    // {
    //   // output("line_strings").set(line_strings);
    //   std::cout << "pushed " << line_strings.size() << " line_string features...\n";
    //   // } else if (geometry_type == wkbPolygon || geometry_type == wkbPolygon25D || geometry_type == wkbPolygonZM || geometry_type == wkbPolygonM) {
    // }
    // else 
    if (polygons.size() > 0)
    {
      std::cout << "pushed " << polygons.size() << " linear_ring features...\n";
    }
    return polygons;
  }
};

std::unique_ptr<VectorReaderInterface> createVectorReaderOGR(projHelperInterface& pjh) {
  return std::make_unique<VectorReaderOGR>(pjh);
};

} // namespace roofer
