#pragma once
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/algorithms/distance.hpp> 

#include <boost/foreach.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <shapefil.h>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


typedef std::pair<box, unsigned> value;

/**************************************************
 * Section 1: DataHandling                        *
 *************************************************/

 /**************************************************
  * Section 1.1: Geometry                          *
  *************************************************/

  /*! \brief WebMercator2WGS84 transforms WebMercator to WGS84.
   *
   *
   * Returns a std::pair(lat,lon) out of two distinct parammeters in mercator.
   */
std::pair<double, double> WebMercator2WGS84(double mercatorY_lat, double mercatorX_lon)
{
	// hacky way of doing it: Convert to WGS84 using some constants
	// and use haversine after that. This can be made faster and 
	// much much more reliable ;-)

	if (abs(mercatorX_lon) < 180 && abs(mercatorY_lat) < 90)
		return std::make_pair(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	if ((abs(mercatorX_lon) > 20037508.3427892) || (abs(mercatorY_lat) > 20037508.3427892))
		return std::make_pair(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());

	double x = mercatorX_lon;
	double y = mercatorY_lat;
	double  num3 = x / 6378137.0;
	double num4 = num3 * 57.295779513082323;
	double num5 = floor((num4 + 180.0) / 360.0);
	double num6 = num4 - (num5 * 360.0);
	double num7 = 1.5707963267948966 - (2.0 * atan(exp((-1.0 * y) / 6378137.0)));
	mercatorX_lon = num6;
	mercatorY_lat = num7 * 57.295779513082323;
	//	 std::cout << "WGS84: "<< mercatorX_lon << ";" << mercatorY_lat << std::endl;
	return (std::make_pair(mercatorY_lat, mercatorX_lon));
}


/*! \brief WebMercatorDistance: Calculate Distance between WebMercator Points
 *
 *
 * Return the distance between two WebMercatorCoordinate pairs by first
 * transforming to WGS84 and using well-known Haversine formula.
 */

template<typename coord>
double WebMercatorDistance(coord p1, coord p2, coord p3, coord p4)
{
	double lon1, lon2, lat1, lat2;
	tie(lat1, lon1) = WebMercator2WGS84(p1, p2);
	tie(lat2, lon2) = WebMercator2WGS84(p3, p4);
	// convert to radians
	lat1 *= (M_PI / 180.0);
	lon1 *= (M_PI / 180.0);
	lat2 *= (M_PI / 180.0);
	lon2 *= (M_PI / 180.0);

	double dlon = lon2 - lon1;
	double dlat = lat2 - lat1;
	double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
	double c = 2 * atan2(sqrt(a), sqrt(1 - a));
	return 6371000.0 * c;
}


/*************************************************
 * Section 1.2: Shape Data Types                 *
 *************************************************/

typedef std::vector<std::vector<float>> polyline;
typedef std::vector<polyline> polycollection;
typedef std::vector<std::vector<float>> pointcollection;

//#include "NewRoadAttrib.hpp"
//#include "NewNodeAttrib.hpp"
//
//typedef std::vector<NewRoadAttrib> newroadattribcollection;
//typedef std::vector<NewNodeAttrib> newnodeattribcollection;


/*************************************************
 * Section 1.3: Data Loading                     *
 *************************************************/
 /*! \brief shapename returns a string name of the SHP type of geometry field
  *
  *
  * Given a SHP type, returns a string representation of the geometry type
  * of the a specific entity.
  */
std::string shapename(size_t type)
{
	switch (type) {
	case 0: return std::string("NULL");
	case 1: return std::string("POINT");
	case 3: return std::string("ARC");
	case 5: return std::string("POLYGON");
	case 8: return std::string("MULTIPOINT");
	default: return std::string("undefined");
	}
}

/*! \brief dbftype returns a string representation of the data type of a DBF field
 *
 *
 * Given a DBF database, the type of the field is returned as a string from the
 * type of a field's integer description.
 */

std::string dbftype(size_t type)
{
	switch (type) {
	case 0: return std::string("FTString");
	case 1: return std::string("FTInteger");
	case 2: return std::string("FTDouble");
	case 3: return std::string("FTLogical");
	default: return std::string("FTundefined");
	}
}

/*! \brief handlePolyline adds a polyline to a collection from a SHP shape
 *
 *
 * Whenever a shape of is read, this function can be used to create a polyline
 * from the geometry in the SHPObject parameter and push it onto the collection
 * given as a parameter
 */
inline void handlePolyline(polycollection& coll, SHPObject* shape)
{
	polycollection::value_type thispline;
	for (size_t i = 0; i < shape->nVertices; i++)
	{
		float x = 0, y = 0, z = 0;
		if (shape->padfX != NULL) x = shape->padfX[i];
		if (shape->padfY != NULL) y = shape->padfY[i];
		if (shape->padfZ != NULL) z = shape->padfZ[i];
		thispline.push_back({ x,y,z });
	}
	coll.push_back(thispline);
}

/*! \brief handlePoint adds a point to a collection from a SHP shape
 *
 * This function reads point data from SHP file objects given by the
 * shape parameter. The resulting point is pushed to a collection in 3D (x,y,z)
 *
 */
inline void handlePoint(pointcollection& coll, SHPObject* shape)
{
	pointcollection::value_type thispoint;
	if (shape->nVertices != 1)
	{
		std::cout << "Point object with more than one vertex. Only adding first" << std::endl;
	}

	if (shape->nVertices == 0)
	{
		std::cout << "Barrier object without point skipped" << std::endl;
		return;
	}

	float x = 0, y = 0, z = 0;
	if (shape->padfX != NULL) x = shape->padfX[0];
	if (shape->padfY != NULL) y = shape->padfY[0];
	if (shape->padfZ != NULL) z = shape->padfZ[0];
	coll.push_back({ x,y,z });
}



/*! \brief importSHP reads a shapefile into collections
 *
 *
 * This function imports a shapefile. The filename is given without the SHP suffix,
 * the collections must support push_back and brace expressions, attributecollections
 * must provide the interface as generated in codegen (checking file format), shapehandler
 * should be either handlePoint for point geometry or handlePolyline for other geometry.
 *
 * if VERY_VERBOSE is defined to one, all shapes are output to the given stream
 */

template<typename polylinecollection, typename attribcollection, typename outstream,
	typename shapehandler>
	void importSHP(std::string filename, polylinecollection& coll,
		attribcollection& attribs,
		shapehandler handler,
		outstream& out = std::cerr, bool verbose = true
	)
{
	SHPHandle	hSHP;
	DBFHandle hDBF;
	int		nShapeType, i, nEntities;
	SHPObject* shape;

	hSHP = SHPOpen((filename + std::string(".shp")).c_str(), "rb");
	hDBF = DBFOpen((filename + std::string(".dbf")).c_str(), "rb");
	if (hSHP == NULL) throw(std::runtime_error("Unable to open Shapefile"));
	if (hDBF == NULL) throw(std::runtime_error("Unable to open DBF "));

	SHPGetInfo(hSHP, &nEntities, &nShapeType, NULL, NULL);
	if (verbose)
		out << "Importing file with " << nEntities << " entities" << std::endl;

	int nDBFRecords = DBFGetRecordCount(hDBF);
	if (verbose)
		out << "Found a DBF with " << nDBFRecords << " entries" << std::endl;

	if (nEntities != nDBFRecords)
	{
		out << "Using DBF and SHP pair of different size: " << filename << std::endl;
	}

	/*And the associated DBF information*/
	size_t fields = DBFGetFieldCount(hDBF);
	if (verbose) {
		for (size_t i = 0; i < fields; i++)
		{
			char name[20];
			size_t type = DBFGetFieldInfo(hDBF, i, name, NULL, NULL);
			std::cout << dbftype(type) << ":" << name << "\n";
		}
		std::cout << std::endl << std::endl;
	}

	// File Type Checking
	if (!attribcollection::value_type::assertCorrectFormat(hDBF)) {
		std::cout << "Error: Wrong DBF format" << std::endl;
	}

	// File Loading loop
	for (i = 0; i < nEntities; i++)
	{
		shape = SHPReadObject(hSHP, i);
		if (shape == NULL) throw(std::runtime_error("unable to read some shape"));
#if VERY_VERBOSE
		std::cout << "Found a shape" << shapename(shape->nSHPType) << "(" << shape->nSHPType << ")" << std::endl;
		std::cout << "ID:" << shape->nShapeId << std::endl;
		std::cout << "numParts: " << shape->nParts << std::endl;
		std::cout << "numVertices:" << shape->nVertices << std::endl;
#endif
		handler(coll, shape);
		SHPDestroyObject(shape);
		/*Read all attributes*/
		typename attribcollection::value_type attrib;
		attrib.readFromID(hDBF, i);
		if (verbose)
			attrib.dump(std::cout);
		attribs.push_back(attrib);
	}
	SHPClose(hSHP);
	DBFClose(hDBF);
	out << "Completed " << nEntities << " entities." << std::endl;
}

/*! \brief importSHPOnlyGeometry
 *
 *
 * Reads an SHP file similar to importSHP, but does not read or parse
 * the associated DBF file. Currently used for polygonal avoidances, as their
 * attributes are actually never used.
 */

template<typename polylinecollection, typename outstream,
	typename shapehandler>
	void importSHPOnlyGeometry(std::string filename, polylinecollection& coll,
		shapehandler handler,
		outstream& out = std::cerr, bool verbose = false
	)
{
	SHPHandle	hSHP;

	int		nShapeType, i, nEntities;
	SHPObject* shape;

	hSHP = SHPOpen((filename + std::string(".shp")).c_str(), "rb");
	if (hSHP == NULL) throw(std::runtime_error("Unable to open Shapefile"));

	SHPGetInfo(hSHP, &nEntities, &nShapeType, NULL, NULL);
	if (verbose)
		out << "Importing file with " << nEntities << " entities" << std::endl;


	// File Loading loop
	for (i = 0; i < nEntities; i++)
	{
		shape = SHPReadObject(hSHP, i);
		if (shape == NULL) throw(std::runtime_error("unable to read some shape"));
#if VERY_VERBOSE
		std::cout << "Found a shape" << shapename(shape->nSHPType) << "(" << shape->nSHPType << ")" << std::endl;
		std::cout << "ID:" << shape->nShapeId << std::endl;
		std::cout << "numParts: " << shape->nParts << std::endl;
		std::cout << "numVertices:" << shape->nVertices << std::endl;
#endif
		handler(coll, shape);
		SHPDestroyObject(shape);
	}
	SHPClose(hSHP);
	out << "Completed " << nEntities << " entities." << std::endl;
}

/*! \brief Interface for importSHP for the case of polylines
 *
 *
 * simplifies calling importSHP.
 */

template<typename polylinecollection, typename attribcollection, typename outstream>
void importSHPPolylines(std::string filename, polylinecollection& coll,
	attribcollection& attribs,
	outstream& out = std::cerr, bool verbose = false
)
{

	importSHP(filename, coll, attribs, handlePolyline, out, verbose);
}

/*! \brief Interface for importSHPOnlyGeometry for polylines
 *
 *
 * simplifies calling importSHP.
 */
template<typename polylinecollection, typename outstream>
void importSHPPolylinesOnlyGeometry(std::string filename, polylinecollection& coll,
	outstream& out = std::cerr, bool verbose = false
)
{

	importSHPOnlyGeometry(filename, coll, handlePolyline, out, verbose);
}

/*! \brief Interface for importSHP for the case of points
 *
 *
 * simplifies calling importSHP.
 */
template<typename collection, typename attribcollection, typename outstream>
void importSHPPoints(std::string filename, collection& coll,
	attribcollection& attribs,
	outstream& out = std::cerr, bool verbose = false
)
{

	importSHP(filename, coll, attribs, handlePoint, out, verbose);
}



/*************************************************
  * Section 1.4: The Document (ShapeCollection)  *
  *************************************************/

  /*! \brief removeExt removes the extension from a filename.
   *
   * The shapefile library expects shapefiles given without extension and refuses to load files
   * given the SHP extension.
   */

std::string removeExt(std::string const& filename)
{
	std::string::const_reverse_iterator pivot
		= std::find(filename.rbegin(), filename.rend(), '.');
	return pivot == filename.rend()
		? filename
		: std::string(filename.begin(), pivot.base() - 1);
}

class ShapeCollection
{
public:

	//polycollection roads; 						///< The road geometry as an STL vector
	//newroadattribcollection aroads;		    ///< The junction attributes
	//bgi::rtree< value, bgi::rstar<16, 4> >
	//	roads_rtree;						///< An R* tree indexing roads

	//polycollection junctions;					///< The junction geometry as an STL vector
	//newnodeattribcollection ajunctions;		///< The junction attributes
	//bgi::rtree< value, bgi::rstar<16, 4> >
	//	junction_rtree;						///< An A* tree indexing junctions

	polycollection polygons;					///< The polygons from the constraint file, as STL vectors
	//std::vector<polygon> obstacles; 			///< The polygons from the constraint file as boost::geometry polygons


public:

	/*! \brief addPolygonalObstacle
	 *
	 *
	 * Adds a polygonal obstacle from the polycollection's valuetype (e.g., an STL
	 * vector<vector<double>> to a boost::geometry polygon (outer, the real polygon,
	 * counter-clockwise oriented, and pushes it to obstacles.
	 *
	 * In short: converts SHAPE data polygon to boost::geometry objects.
	 */

	//void addPolygonalObstacle(polycollection::value_type& in)
	//{
	//	polygon p;
	//	for (auto& x : in)
	//	{
	//		p.outer().push_back(point(x[0], x[1]));
	//	}
	//	obstacles.push_back(p);
	//	/*		BOOST_FOREACH(polygon const& x, obstacles)
	//				std::std::cout << bg::wkt<polygon>(x) << std::std::endl;*/
	//}

	/*! \brief getMBR returns the bounding box of the junctinos
	 *
	 *
	 * This method returns the bounding rectangle of junctions into
	 * referenced parameters left, right, bottom and up. It is mainly
	 * used for OpenGL GUI zoomFit() functionality
	 *
	 */

	//void getMBR(double& l, double& r, double& b, double& t)
	//{
	//	l = b = std::numeric_limits<double>::infinity();
	//	r = t = -std::numeric_limits<double>::infinity();
	//	for (auto& j : junctions)
	//	{
	//		double x = j[0][0];
	//		double y = j[0][1];
	//		if (x < l) l = x;
	//		if (x > r) r = x;
	//		if (y < b) b = y;
	//		if (y > t) t = y;
	//	}
	//}

	/*! \brief fillSpatialIndizes()
	 *
	 *
	 * Takes all roads and junctions and fills the repspective R-trees.
	 * The roads are indexed as points, that is nearest point from any road is
	 * directly answered as nearest predicate on road R-tree resulting in the
	 * road index to which the point belongs. Note that this is not actually nearest
	 * on the road given as a linestring.
	 *
	 * Junctions are points and therefore obviously indexed as points.
	 */

	//void fillSpatialIndizes()
	//{
	//	/*Fill roads by creating temporary polygons*/
	//	for (size_t i = 0; i < roads.size(); i++)
	//	{
	//		polygon poly;
	//		for (auto p : roads[i])
	//		{
	//			box b(point(p[0], p[1]), point(p[0], p[1]));
	//			roads_rtree.insert(std::make_pair(b, i));
	//		}
	//	}

	//	for (size_t i = 0; i < junctions.size(); i++)
	//	{
	//		box b(point(junctions[i][0][0], junctions[i][0][1]),
	//			point(junctions[i][0][0], junctions[i][0][1]));

	//		junction_rtree.insert(std::make_pair(b, i));
	//	}
	//}



	/*! \brief nearestJunction returns the index of the nearest junction given coordinates
	 *
	 *
	 * The nearest junction is calculated using Euclidean geometry. Mainly used to support
	 * mouse clicking in the GUI, therefore Euclidean geometry is what users expect. This method
	 * interrupts with SEGFAULT, if junction_rtree should be empty.
	 */
	//size_t nearestJunction(double x, double y)
	//{
	//	std::vector<value> result_n;
	//	junction_rtree.query(bgi::nearest(point(x, y), 1), std::back_inserter(result_n));
	//	return result_n[0].second;
	//}



	/*! \brief nearestRoads returns the index of the k nearest road points
	 *
	 *
	 * This method interrupts with SEGFAULT, if roads_rtree should be empty.
	 */
	/*std::vector<size_t> nearestRoads(double x, double y, size_t k)
	{
		std::vector<value> result_n;
		roads_rtree.query(bgi::nearest(point(x, y), k), std::back_inserter(result_n));
		std::vector<size_t> ret;
		for (auto a : result_n) {
			ret.push_back(a.second);
		}
		return ret;
	}*/

	/*std::vector<size_t> junctionsOnRoad(size_t road)
	{
		std::vector<size_t> ret;
		for (size_t i=0; i < roads[road].size(); i++)
		{
			auto junction=nearestJunction(roads[road][i][0],roads[road][i][1]);
			if (d(roads[road][i], junctions[junction][0]) < 0.1)
			{
				ret.push_back(junction);
			}
		}
		return ret;
	}
	bool is_junction(size_t road, size_t idx)
	{
		auto junction=nearestJunction(roads[road][idx][0],roads[road][idx][1]);
		return (d(roads[road][idx], junctions[junction][0]) < 0.1);
	}
	std::pair<size_t,size_t> find_next_junction(size_t road, size_t idx)
	{
		for (size_t i=idx; i < roads[road].size(); i++)
		{
			auto junction=nearestJunction(roads[road][i][0],roads[road][i][1]);
			if (d(roads[road][i], junctions[junction][0]) < 0.1)
			{
				return std::make_pair(junction,i); // junction index and road index
			}
		}
		return std::make_pair(junctions.size(),roads[road].size());
	}


	double roadWeight(size_t road, size_t start, size_t end, size_t valuation=VALUATION_DISTANCE)
	{
		double ret = 0;
		for (size_t i=start; i < end; i++)
		{
			switch(valuation){
				case VALUATION_DISTANCE:
					ret += d(roads[road][i],roads[road][i+1]);

					break;
				case VALUATION_TIME:

					break;
				default:
					std::cout << "Unknown valuation" << std::endl;
			}

		}
		//std::cout << "#?" << d(roads[road][start],roads[road][end]) << std::endl;
		//std::cout << road <<"," << start<< "," << end <<"=="  << ret << std::endl;
		return ret;
	}
	*/

	/*! \brief LoadFiles loads road and node from file names
	 *
	 *
	 * First, loads given roads file (removing extension, if present)
	 * Second, loads given junctions file (removing extension, if present)
	 * Finally, creates spatial indizes
	 */
	//void LoadFiles(std::string roadfile, std::string junctionfile)
	//{

	//	std::cout << "Step 1: Load roads    \t";
	//	importSHPPolylines(removeExt(roadfile), roads, aroads, std::cout);

	//	std::cout << "Step 2: Load junctions\t";
	//	importSHPPolylines(removeExt(junctionfile), junctions, ajunctions, std::cout);

	//	fillSpatialIndizes();
	//}



	///*! \brief LoadDirectory loads road and node files from a directory assuming default names
	// *
	// *
	// * First, loads sfo_roads.* SHP and DBF file into roads and aroads,
	// * Second, loads sfo_nodes.* into junctions and ajunctions
	// * Finally, creates spatial indizes
	// */
	//void LoadDirectory(std::string filebase)
	//{
	//	std::cout << "Loading from " << filebase << std::endl;
	//	std::cout << "Step 1: Load roads    \t";
	//	importSHPPolylines((filebase + "/sfo_roads"), roads, aroads, std::cout);

	//	std::cout << "Step 2: Load junctions\t";
	//	importSHPPolylines((filebase + "/sfo_nodes"), junctions, ajunctions, std::cout);
	//	//std::cout << "Step 5: Spatial Indizes"<< std::endl;
	//	fillSpatialIndizes();
	//}


	/*! \brief LoadPolygons
	 *
	 *
	 * Loads a file (extension is removed, if it was given) containing polygonal data
	 * Note that it clears polygons, the polygon set last read from a file.
	 */

	void LoadPolygons(std::string fname)
	{
		polygons.clear();
		importSHPPolylinesOnlyGeometry(removeExt(fname), polygons, std::cout);
		std::cout << "Found " << polygons.size() << "polygons" << std::endl;
	}

};