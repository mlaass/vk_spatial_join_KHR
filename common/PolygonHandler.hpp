#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/regex.hpp>
#include <algorithm>
#include <random>

#include <cmath>

#define PI 3.14159265358979323846

#include "./my_stdint.h"



namespace bg = boost::geometry;
namespace transform = boost::geometry::strategy::transform;

typedef bg::model::d2::point_xy<float> point;
typedef bg::model::linestring<point> linestring;
typedef bg::model::polygon<point> polygon;
typedef bg::model::multi_polygon<polygon> multi_polygon;
typedef bg::model::box<point> box;


double median(std::vector<int> &v) {
  size_t n = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + n, v.end());
  int vn = v[n];
  if (v.size() % 2 == 1) {
    return vn;
  } else {
    std::nth_element(v.begin(), v.begin() + n - 1, v.end());
    return 0.5 * (vn + v[n - 1]);
  }
}

#include "shapeloader.h"
class PolygonHandler
{
private:
  box _poly_bounds;

public:
  std::vector<polygon> polys;
  std::vector<point> points;
  std::vector<int> points_indices;
  std::vector<int> polys_indices;

  std::vector<int> polys_sizes;

  float2 *points_array = NULL;
  size_t POINTS_BUFFER_SIZE = 4294967296; // 2^32

  PolygonHandler(size_t points_buffer_size=4294967296):POINTS_BUFFER_SIZE(points_buffer_size) {

  }
  ~PolygonHandler()
  {
    if (points_array != NULL)
      delete points_array;
    points_array = NULL;
  }
  void makePointsArray()
  {
    if (points_array)
    {
      delete points_array;
    }
    size_t sz = points.size();
    sz = ((sz / 8) * 8) + 8;
    points_array = new float2[sz];
    for (auto i = 0; i < points.size(); ++i)
    {
      auto p = points[i];
      float x = p.x();
      float y = p.y();
      points_array[i] = {x, y};
    }
  }
  size_t getPointsIterations(){
    if(points.size() > POINTS_BUFFER_SIZE){
      return points.size() / POINTS_BUFFER_SIZE;
    }
    return 1;
  }
  size_t getPointsCount(size_t iteration=0){
    size_t sz = POINTS_BUFFER_SIZE;
    if(points.size() > POINTS_BUFFER_SIZE){
      sz= std::min(POINTS_BUFFER_SIZE,
                      points.size() - (iteration * POINTS_BUFFER_SIZE));
    }else{
      sz = points.size();
    }
    //sz = ((sz / 8) * 8) + 8;
    //std::cout << "getPointsCount(" << iteration << ") = " << sz << std::endl;
    return sz;
  }

   float2 * getPointsNBufferOffset(size_t iteration){
     return &points_array[POINTS_BUFFER_SIZE * iteration];
  }

  box getBounds()
  {
    float min_x = std::numeric_limits<float>::max(),
          min_y = std::numeric_limits<float>::max(),
          max_x = std::numeric_limits<float>::min(),
          max_y = std::numeric_limits<float>::min();
    for (auto i = 0; i < points.size(); ++i)
    {
      auto p = points[i];
      float x = p.x();
      float y = p.y();

      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);
    }
    return box{{min_x, min_y}, {max_x, max_y}};
  }

  void printPolyStats() {

    int min = std::numeric_limits<int>::max();
    int max = std::numeric_limits<int>::min();
    for(auto it :polys_sizes){
      min = std::min(min, it);
      max = std::max(max, it);
    }
    auto med = median(polys_sizes);
    std::cout << "min: " << min << " max: "<< max << " median: " << med << std::endl;
  }

  box getPolyBounds()
  {
    float min_x = std::numeric_limits<float>::max(),
          min_y = std::numeric_limits<float>::max(),
          max_x = std::numeric_limits<float>::min(),
          max_y = std::numeric_limits<float>::min();
    for (auto i = 0; i < polys.size(); ++i)
    {
      auto p = polys[i];
      box b;
      bg::envelope(p, b);

      min_x = std::min(min_x, b.min_corner().x());
      min_y = std::min(min_y, b.min_corner().y());
      max_x = std::max(max_x, b.max_corner().x());
      max_y = std::max(max_y, b.max_corner().y());
    }
    return box{{min_x, min_y}, {max_x, max_y}};
  }

  void makeCircle(int segments, float radius, float center_x, float center_y)
  {
    std::cout << "makeCircle: " << segments << ", "
              << radius << ", "
              << center_x << ", "
              << center_y << "...";
    float alpha = 2.f * PI / (float)segments;
    polygon poly = polygon();
    for (auto i = 0; i < segments; i++)
    {
      auto x = center_x + radius * sinf(alpha * (float)i);
      auto y = center_y + radius * cosf(alpha * (float)i);
      //std::cout << "p(" << x << ", " << y << "),";

      bg::append(poly.outer(), point(x, y));
    }
    // std::cout << "\n";
    polys.push_back(poly);
    polys_indices.push_back((int)polys.size());
    _poly_bounds = getPolyBounds();
    std::cout << " done!" << std::endl;
  }

  void makePoint(float x, float y)
  {
    points.push_back(point(x, y));
    points_indices.push_back((int)points.size());
    makePointsArray();
  }
  void makeDuplications(int num, float translation_scale)
  {
    bool dup_polys = polys.size() > 0;
    bool dup_points = points.size() > 0;
    std::cout << "making duplications: " << num << std::endl;
    for (int d = 0; d < num; d++)
    {
      std::cout << "before duplication: " << d
                << ", polys: " << polys.size()
                << ", points: " << points.size()
                << std::endl;

      auto dir = d % 2;
      auto bb = getBounds();
      float trns_x = bb.max_corner().get<0>() - bb.min_corner().get<0>();
      float trns_y = 0;
      if (dir != 0)
      {
        trns_y = bb.max_corner().get<1>() - bb.min_corner().get<1>();
        trns_x = 0;
      }
      std::cout << "trans_x: " << trns_x << ", trans_y: " << trns_y
                << std::endl;

      trns_x *= translation_scale;
      trns_y *= translation_scale;
      transform::translate_transformer<float, 2, 2>
          translate(trns_x, trns_y);

      auto sz_polys = polys.size();
      auto sz_points = points.size();
      if (dup_polys)
      {
        polys_indices.resize(sz_polys * 2);
        polys.resize(sz_polys * 2);

        for (auto i = 0; i < sz_polys; i++)
        {
          polys_indices[sz_polys + i] = polys_indices[i] + sz_polys;
        }
        for (auto i = 0; i < sz_polys; i++)
        {
          auto p = polygon(polys[i]);
          boost::geometry::transform(polys[i], p, translate);
          polys[sz_polys + i] = p;
        }
      }

      if (dup_points)
      {
        points_indices.resize(sz_points * 2);
        points.resize(sz_points * 2);
        for (auto i = 0; i < sz_points; i++)
        {
          points_indices[sz_points + i] = points_indices[i] + sz_points;
        }
        for (auto i = 0; i < sz_points; i++)
        {
          auto p = point(points[i]);
          boost::geometry::transform(points[i], p, translate);
          points[sz_points + i] = p;
        }
      }
      std::cout << "finished duplication: " << d
                << ", polys: " << polys.size()
                << ", points: " << points.size()
                << std::endl;
      // std::stringstream ss;
      // ss << "test_dup_points_" << d << ".csv";
      // savePointsIndexed(ss.str());
      // ss = std::stringstream();
      // ss << "test_dup_voronoi_" << d << ".csv";
      // savePolygonsIndexed(ss.str());
    }
    if (dup_points)
    {
      makePointsArray();
    }
    if (dup_polys)
    {
      _poly_bounds = getPolyBounds();
    }
  }

  template <typename outputiterator>
  void extract3DGeometry(outputiterator out, size_t interval_start, size_t interval_end)
  {
      size_t end = min(interval_end, polys.size());
      size_t vertex_count = 0;
    for (size_t i = interval_start; i < end; i++)
    {
      auto emit = [&out](const point &a, const point &b, float height) {
        // std::cout << "Emitting " << bg::get<0>(a) << ";" << bg::get<1>(a) << "=>" << bg::get<0>(b) << ";" << bg::get<1>(b) << std::endl;

        *out++ = {bg::get<0>(a), bg::get<1>(a), 0.0f};
        *out++ = {bg::get<0>(b), bg::get<1>(b), 0.0f};
        *out++ = {bg::get<0>(b), bg::get<1>(b), height};

        *out++ = {bg::get<0>(b), bg::get<1>(b), height};
        *out++ = {bg::get<0>(a), bg::get<1>(a), height};
        *out++ = {bg::get<0>(a), bg::get<1>(a), 0.0f};
      }; // emit
      const auto &p = bg::exterior_ring(polys[i]);
      if (p.size() < 3)
        throw(std::runtime_error("Bad polygon"));

      float height = (float)i + 1;
      for (size_t j = 1; j < p.size(); j++) {
        emit(p[j - 1], p[j], height);
        vertex_count++;

      }
    }
    std::cout << " vertex_count: " << vertex_count<< std::endl;
  }

  template <typename outputiterator>
  size_t extract3DGeometryVertexStream(outputiterator out, size_t poly_start,size_t max_vertices)
  {
    //size_t end = min(interval_end, polys.size());
    size_t vertex_count = 0;
    size_t i = poly_start;
    for ( i = poly_start; i < polys.size(); i++)
    {
        auto emit = [&out](const point &a, const point &b, float height) {
            // std::cout << "Emitting " << bg::get<0>(a) << ";" << bg::get<1>(a) << "=>" << bg::get<0>(b) << ";" << bg::get<1>(b) << std::endl;

            *out++ = {bg::get<0>(a), bg::get<1>(a), 0.0f};
            *out++ = {bg::get<0>(b), bg::get<1>(b), 0.0f};
            *out++ = {bg::get<0>(b), bg::get<1>(b), height};

            *out++ = {bg::get<0>(b), bg::get<1>(b), height};
            *out++ = {bg::get<0>(a), bg::get<1>(a), height};
            *out++ = {bg::get<0>(a), bg::get<1>(a), 0.0f};
        }; // emit

        const auto &p = bg::exterior_ring(polys[i]);
        if (p.size() < 3)
        throw(std::runtime_error("Bad polygon"));

        float height = (float)i + 1;

        if (vertex_count + p.size() > max_vertices)
            break;
        else
            for (size_t j = 1; j < p.size(); j++) {
                emit(p[j - 1], p[j], height);
                vertex_count++;
            }
    }
    std::cout << " vertex_count: " << vertex_count << " polygon index: " << i<< std::endl;
    return i;
  }

  void readPolygonsIndexed(const std::string filename)
  {
    try
    {

      std::cout << "readPolygonsIndexed: " << filename << "...";
      // used to split the file in lines
      const boost::regex linesregx("\\r\\n|\\n\\r|\\n|\\r");

      // used to split each line to tokens, assuming ',' as column separator
      const boost::regex fieldsregx(",(?=(?:[^\"]*\"[^\"]*\")*(?![^\"]*\"))");

      std::ifstream csvFile;
      csvFile.open(filename.c_str());

      if (!csvFile.is_open())
      {
        std::cout << "Path Wrong!!!!" << std::endl;
        exit(EXIT_FAILURE);
      }

      std::string line;
      std::vector<std::string> vec;
      std::getline(csvFile, line); // skip the 1st line

      while (std::getline(csvFile, line))
      {
        if (line.empty()) // skip empty lines:
        {
          //cout << "empty line!" << endl;
          continue;
        }

        // Split line to tokens
        boost::sregex_token_iterator ti(line.begin(), line.end(), fieldsregx, -1);
        boost::sregex_token_iterator end2;

        std::vector<std::string> row;
        while (ti != end2)
        {
          std::string token = ti->str();
          ++ti;
          row.push_back(token);
        }
        if (line.back() == ',')
        {
          // last character was a separator
          row.push_back("");
        }

        {
          polygon p;
          auto s = row[0];
          s.erase(remove(s.begin(), s.end(), '\"'), s.end());
          // std::cout << s << std::endl;
          bg::read_wkt(s, p);
          bg::correct(p);
          polys.push_back(p);
        }
        {
          auto ss = row[1];
          ss.erase(remove(ss.begin(), ss.end(), '\"'), ss.end());
          // std::cout << ss << std::endl;
          polys_indices.push_back(std::stoi(ss));
        }
      }
      _poly_bounds = getPolyBounds();
      std::cout << " done!" << std::endl;
    }
    catch (std::exception &e)
    {
      std::cerr << "Reading Poly with exception: " << e.what() << "\n";
      throw(std::runtime_error(std::string("Caught ") + e.what()));
    }
  }

  void readMultiPolygonsIndexed(const std::string filename)
  {
    try
    {

      std::cout << "readMultiPolygonsIndexed: " << filename << "...";
      // used to split the file in lines
      const boost::regex linesregx("\\r\\n|\\n\\r|\\n|\\r");

      // used to split each line to tokens, assuming ',' as column separator
      const boost::regex fieldsregx(",(?=(?:[^\"]*\"[^\"]*\")*(?![^\"]*\"))");

      std::ifstream csvFile;
      csvFile.open(filename.c_str());

      if (!csvFile.is_open())
      {
        std::cout << "Path Wrong!!!!" << std::endl;
        exit(EXIT_FAILURE);
      }

      std::string line;
      std::vector<std::string> vec;
      std::getline(csvFile, line); // skip the 1st line

      while (std::getline(csvFile, line))
      {
        if (line.empty()) // skip empty lines:
        {
          //cout << "empty line!" << endl;
          continue;
        }

        // Split line to tokens
        boost::sregex_token_iterator ti(line.begin(), line.end(), fieldsregx, -1);
        boost::sregex_token_iterator end2;

        std::vector<std::string> row;
        while (ti != end2)
        {
          std::string token = ti->str();
          ++ti;
          row.push_back(token);
        }
        if (line.back() == ',')
        {
          // last character was a separator
          row.push_back("");
        }


        multi_polygon mp;
        auto s = row[0];
        s.erase(remove(s.begin(), s.end(), '\"'), s.end());
        //std::cout << s << std::endl;
        bg::read_wkt(s, mp);
        bg::correct(mp);

        auto ss = row[row.size() - 1];
        ss.erase(remove(ss.begin(), ss.end(), '\"'), ss.end());
        //std::cout << ss << std::endl;
        auto index = std::stoi(ss);

        for (auto i = 0; i < mp.size(); i++) {
          polygon p = mp[i];
          polys.push_back(p);
          polys_indices.push_back(index);
        }
      }
      _poly_bounds = getPolyBounds();
      std::cout << " done!" << std::endl;
    }
    catch (std::exception &e)
    {
      std::cerr << "Reading MultiPoly with exception: " << e.what() << "\n";
      throw(std::runtime_error(std::string("Caught ") + e.what()));
    }
  }
  void readPointsIndexed(const std::string filename)
  {
    std::cout << "readPointsIndexed: " << filename << "...";

    // used to split the file in lines
    const boost::regex linesregx("\\r\\n|\\n\\r|\\n|\\r");

    // used to split each line to tokens, assuming ',' as column separator
    const boost::regex fieldsregx(",(?=(?:[^\"]*\"[^\"]*\")*(?![^\"]*\"))");

    std::ifstream csvFile;
    csvFile.open(filename.c_str());

    if (!csvFile.is_open())
    {
      std::cout << "Path Wrong!!!!" << std::endl;
      exit(EXIT_FAILURE);
    }

    std::string line;
    std::vector<std::string> vec;
    std::getline(csvFile, line); // skip the 1st line

    while (std::getline(csvFile, line))
    {
      if (line.empty()) // skip empty lines:
      {
        //cout << "empty line!" << endl;
        continue;
      }

      // Split line to tokens
      boost::sregex_token_iterator ti(line.begin(), line.end(), fieldsregx, -1);
      boost::sregex_token_iterator end2;

      std::vector<std::string> row;
      while (ti != end2)
      {
        std::string token = ti->str();
        ++ti;
        row.push_back(token);
      }
      if (line.back() == ',')
      {
        // last character was a separator
        row.push_back("");
      }

      {
        point p;
        auto s = row[0];
        s.erase(remove(s.begin(), s.end(), '\"'), s.end());
        //std::cout << s << std::endl;
        bg::read_wkt(s, p);
        points.push_back(p);
      }
      {
        auto ss = row[1];
        ss.erase(remove(ss.begin(), ss.end(), '\"'), ss.end());
        //std::cout << ss << std::endl;
        points_indices.push_back(std::stoi(ss));
      }
    }
    makePointsArray();
    std::cout << " done!" << std::endl;
  }

  void readPointsBinary(const std::string filename)
  {
    std::cout << "readPointsBinary: " << filename << "...";

    // used to split the file in lines
    const boost::regex linesregx("\\r\\n|\\n\\r|\\n|\\r");

    // used to split each line to tokens, assuming ',' as column separator
    const boost::regex fieldsregx(",(?=(?:[^\"]*\"[^\"]*\")*(?![^\"]*\"))");

    std::ifstream binFile;
    binFile.open(filename.c_str(), std::ios::binary);

    if (!binFile.is_open())
    {
      std::cout << "Path Wrong!!!!" << std::endl;
      exit(EXIT_FAILURE);
    }

    binFile.unsetf(std::ios::skipws);

    std::streampos file_size;
    binFile.seekg(0, std::ios::end);
    file_size = binFile.tellg();
    binFile.seekg(0, std::ios::beg);

    points.resize(file_size / sizeof(float2));
    points_indices.resize(file_size / sizeof(float2));

    std::vector<char> vec(file_size);

    vec.insert(vec.begin(),
               std::istream_iterator<char>(binFile),
               std::istream_iterator<char>());

    //std::transform(vec.begin(), vec.end(), std::back_inserter( points ),
    //               [](float2 c)->point{ return point(c[0], c[1]) });
    auto src = (char*) new char[vec.size()];
    std::move(vec.begin(), vec.end(), src);
    points_array = (float2*) src;
    for (auto i = 0; i < points.size(); i++){
      points_indices[i] = i + 1;
    }

      //makePointsArray();
      std::cout << " done!" << std::endl;
  }

  void savePointsIndexed(const std::string &filename)
  {

    std::ofstream wkt_file(filename);

    // Send the column name to the stream
    wkt_file << "WKT,id\n";

    // Send data to the stream
    for (auto i = 0; i < points.size(); ++i)
    {
      wkt_file << "\"" << bg::wkt(points[i]) << "\",\"" << points_indices[i] << "\"\n";
    }
  }

  void savePolygonsIndexed(const std::string &filename)
  {

    std::ofstream wkt_file(filename);

    // Send the column name to the stream
    wkt_file << "WKT,id\n";

    // Send data to the stream
    for (auto i = 0; i < polys.size(); ++i)
    {
      wkt_file << "\"" << bg::wkt(polys[i]) << "\",\"" << polys_indices[i] << "\"\n";
    }
  }

  void resizePoints(long num)
  {
    std::cout << "resizePoints: " << num << std::endl;
    auto sz_points = points.size();
    points_indices.resize(num);
    points.resize(num);
    for (auto i = sz_points; i < num; i++)
    {
      points_indices[i] = points_indices[i - sz_points] + sz_points;
    }
    makePointsArray();
  }

  void shufflePointsArray()
  {
    std::cout << "shufflePointsArray...";
    srand(std::time(0));
    int slices = min((int)500, (int)(points.size()/100));
    long gap = points.size() / slices;
    int shuffle_size = min(512, gap/3);
    int off = rand() % (gap - ((shuffle_size*3)/2));

    for (long i = 0; i < slices; i++){
      long d = off + (i * gap);
      std::random_shuffle(&points_array[d], &points_array[shuffle_size + d]);
    }
  }

  void randomizePointsArray()
  {
    std::cout << "randomizePointsArray...";

    auto min_x = _poly_bounds.min_corner().x() * 0.999;
    auto min_y = _poly_bounds.min_corner().y() * 0.999;
    auto max_x = _poly_bounds.max_corner().x() * 0.999;
    auto max_y = _poly_bounds.max_corner().y() * 0.999;
    // std::cout << "bounds("
    //           << min_x << ", "
    //           << min_y << ", "
    //           << max_x << ", "
    //           << max_y << ", "
    //           << ")... ";

    std::random_device rd;                                     // obtain a random number from hardware
    std::mt19937 gen(rd());                                    // seed the generator
    std::uniform_real_distribution<float> rnd_x(min_x, max_x); // define the range
    std::uniform_real_distribution<float> rnd_y(min_y, max_y); // define the range

    for (auto i = 0; i < points.size(); i++)
    {
      float x = rnd_x(gen);
      float y = rnd_y(gen);
      // if (i < 10)
      //   std::cout << "p(" << x << ", " << y << ") \n";

      points_array[i] = {x, y};
    }
    std::cout << " done!" << std::endl;
  }

  void readShapefile(const std::string filename)
  {
    try
    {

      std::cout << "readShapefile: " << filename << "...\n";
      ShapeCollection sc;
      sc.LoadPolygons(filename);
      std::cout << "polygons: " << sc.polygons.size() << " {";
      long vertices = 0;
      int id = 0;

      for (auto p = std::begin(sc.polygons); p != std::end(sc.polygons); ++p ){
        polygon pol;
        size_t pvertices = 0;

        for (auto pp = std::begin(*p); pp != std::end(*p); ++pp) {
            bg::append(pol.outer(), point( (*pp)[0], (*pp)[1]) );
            vertices++;
            pvertices++;
        }

        polys.push_back(pol);
        polys_indices.push_back(++id);
        polys_sizes.push_back(pvertices);

        std::cout << id-1<< ": " << pvertices << ", ";
      }
      std::cout <<"}"<< std::endl << " vertices: " << vertices << " stats: ";
      printPolyStats();


      _poly_bounds = getPolyBounds();
      std::cout << " done!" << std::endl;
    }
    catch (std::exception &e)
    {
      std::cerr << "Reading Shapefile with exception: " << e.what() << "\n";
      throw(std::runtime_error(std::string("Caught ") + e.what()));
    }
  }
};