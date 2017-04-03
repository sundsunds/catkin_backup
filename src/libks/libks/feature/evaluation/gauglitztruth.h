//----------------------------
// Ground truth coordinates
//----------------------------

struct point3D
{
  point3D(float a, float b, float c) : x(a), y(b), z(c) {}
  float x, y, z;
};

struct point2D
{
  point2D(float a, float b) : x(a), y(b) {}
  float x, y;
};

struct rect2D
{
  rect2D(float a, float b, float c, float d) : x1(a), y1(b), x2(c), y2(d) {}
  float x1, y1, x2, y2;
};

// 3D "world" coordinates

const float paper_width  = 11.0f; // inch
const float paper_height =  8.5f; // inch

const float marker_margin = 1.0f; // margin between markers and paper, inch

const float dist_markers_x = paper_width  + 2*marker_margin; // inch
const float dist_markers_y = paper_height + 2*marker_margin; // inch

const float outer_margin = 0.5f; // margin around the markers, inch

const point3D corners_worldcoordinates[4] = { // in mm
	point3D(-165.1f,-133.35f, 0.0f), point3D(-165.1f, 133.35, 0.0f),
	point3D( 165.1f, 133.35f, 0.0f), point3D( 165.1f,-133.35, 0.0f)
};

// warped 2D image coordinates

const float DPI = 25.4f; // i.e. 1 pixel = 1 mm
const int dst_w = int((dist_markers_x+2*outer_margin)*DPI); // size of warped image
const int dst_h = int((dist_markers_y+2*outer_margin)*DPI);

const float x1__ =  outer_margin*DPI;
const float x2__ = (outer_margin + dist_markers_x)*DPI;
const float y1__ =  outer_margin*DPI;
const float y2__ = (outer_margin + dist_markers_y)*DPI;

const point2D dst_corners[4] = { // coordinates of where the corners should be warped to
	point2D(x1__,y1__), point2D(x1__,y2__),
	point2D(x2__,y2__), point2D(x2__,y1__)
};

// area of paper
const float paper_margin = 0.1f; // inner margin (inch)
const rect2D planarROI(
	(outer_margin+marker_margin+paper_margin)*DPI,
	(outer_margin+marker_margin+paper_margin)*DPI,
	(outer_margin+marker_margin+paper_width -paper_margin)*DPI,
	(outer_margin+marker_margin+paper_height-paper_margin)*DPI
);
const point2D planarROI_p[4] = { // same coordinates as array of points
	point2D(planarROI.x1,planarROI.y1), point2D(planarROI.x2,planarROI.y1),
	point2D(planarROI.x2,planarROI.y2), point2D(planarROI.x1,planarROI.y2)
};

// area of texture allowed to use
const float texture_margin = 1.5f; // inner margin (inch)
const rect2D textureROI(
	(outer_margin+marker_margin+texture_margin)*DPI,
	(outer_margin+marker_margin+texture_margin)*DPI,
	(outer_margin+marker_margin+paper_width -texture_margin)*DPI,
	(outer_margin+marker_margin+paper_height-texture_margin)*DPI
);
const point2D textureROI_p[4] = { // same coordinates as array of points
	point2D(textureROI.x1,textureROI.y1), point2D(textureROI.x2,textureROI.y1),
	point2D(textureROI.x2,textureROI.y2), point2D(textureROI.x1,textureROI.y2)
};

