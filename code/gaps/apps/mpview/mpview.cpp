// Source file for the mp viewer program



////////////////////////////////////////////////////////////////////////
// Include files 
////////////////////////////////////////////////////////////////////////

#include <unordered_map>

#include "R3Graphics/R3Graphics.h"
#include "RGBD/RGBD.h"
#include "fglut/fglut.h"
#include "mp.h"
#include <png/png.h>


////////////////////////////////////////////////////////////////////////
// Global variables
////////////////////////////////////////////////////////////////////////

// Program variables

static char *input_house_filename = NULL;
static char *input_scene_filename = NULL;
static char *input_mesh_filename = NULL;
static char *input_segments_filename = NULL;
static char *input_objects_filename = NULL;
static char *input_categories_filename = NULL;
static char *input_configuration_filename = NULL;
static char *input_ssa_filename = NULL;
static char *input_ssb_filename = NULL;
static char *output_house_filename = NULL;
static char *output_image_filename = NULL;
static char* output_images_path = NULL;
static R3Vector initial_camera_towards(0, 0, -1);
static R3Vector initial_camera_up(0,1,0);
static R3Point initial_camera_origin(0,0,0);
static RNBoolean initial_camera = FALSE;
static RNRgb background(0,0,0);
static int print_verbose = 0;
static int batch = 0;
static char* match_meshes = nullptr;

// Application variables

static MPHouse *house = NULL;
static R3Viewer *viewer = NULL;
static MPRegion *selected_region = NULL;
static MPObject *selected_object = NULL;
static MPImage *selected_image = NULL;
static MPImage *snap_image = NULL;
static int snap_image_index = -1;


// Draw flag variables

static RNFlags level_draw_flags = MP_DRAW_DEPICTIONS;
static RNFlags region_draw_flags = MP_DRAW_FACES | MP_DRAW_DEPICTIONS;
static RNFlags object_draw_flags = MP_SHOW_OBJECTS | MP_SHOW_SEGMENTS; // | MP_DRAW_BBOXES;
static RNFlags image_draw_flags = MP_DRAW_DEPICTIONS;
static RNFlags panorama_draw_flags = MP_DRAW_DEPICTIONS;
static RNFlags mesh_draw_flags = MP_DRAW_VERTICES;
static RNFlags scene_draw_flags = MP_SHOW_SCENE | MP_DRAW_FACES;
static RNFlags color_scheme = MP_COLOR_BY_OBJECT | MP_COLOR_BY_LABEL;


// Display variables

static int show_clip_box = 1;
static int show_axes = 0;
static int show_backfacing = 0;
static R3Box clip_box = R3null_box;
static R3Point center(0, 0, 0);


// GLUT variables 

static int GLUTwindow = 0;
static int GLUTwindow_width = 640;
static int GLUTwindow_height = 512;
static int GLUTmouse[2] = { 0, 0 };
static int GLUTbutton[3] = { 0, 0, 0 };
static int GLUTmouse_drag = 0;
static int GLUTmodifiers = 0;



////////////////////////////////////////////////////////////////////////
// Info functions
////////////////////////////////////////////////////////////////////////

static void
PrintUsage(void)
{
  printf("Usage: mpview [inputfiles] [options]\n");
  printf("Options:\n");
  printf("  -input_house <filename> : input region segmentations file (e.g., xxx/region_segmentations/xxx.house)\n");
  printf("  -input_scene <filename> : input textured mesh (e.g., xxx/matterport_mesh/*/*.obj)\n");
  printf("  -input_mesh <filename> : input poisson mesh (e.g., xxx/poisson_meshes/xxx_11.ply)\n");
  printf("  -input_categories <filename> : input categories tsv file (e.g., metadata/*.tsv)\n");
  printf("  -input_segments <filename> : input json file with face segments (e.g., xxx/object_segmentations/*.fsegs.json)\n");
  printf("  -input_objects <filename> : input json file with objects and labels (e.g., xxx/object_segmentations/*.semseg.json)\n");
  printf("  -input_configuration <filename> : input file with images and panorama (e.g., xxx/undistorted_camera_parameters/xxx.conf)\n");
  printf("  -output_image <filename> : save an image to <filename> and exit\n");
  printf("  -background <r> <g> <b> : background color (with each component in [0.0-1.0])\n");
  printf("  -window <width> <height> : window size in pixels\n");
  printf("  -camera <ex> <ey> <ez> <tx> <ty> <tz> <ux> <uy> <uz> : initial camera extrinsics\n");
  printf("  -batch : exit without starting interactive viewer\n");
  printf("  -write_images <output folder> : writes out images of the house, real, synthetic image and segmentation.\n");
  printf("  -v : print verbose (recommended)\n");
  printf("\n");
  printf("Typical usage for viewing house segmentations:\n");
  printf("  cd scans/17DRP5sb8fy (or any other house)\n");
  printf("  mpview -input_house house_segmentations/*.house -v   OR \n");
  printf("  mpview -input_house house_segmentations/*.house -input_scene matterport_mesh/*/*.obj -v   OR\n");
  printf("  mpview -input_house house_segmentations/*.house -input_mesh house_segmentations/*.ply -v\n");
  printf("\n");
}



static void
PrintCommands(void)
{
  // Print info about the user interface
  printf("\n");
  printf("Camera control:\n");
  printf("    Left-mouse drag = rotate\n");
  printf("    Right-mouse drag = pan\n");
  printf("    Middle-mouse drag = zoom\n");
  printf("    Thumbwheel = zoom\n");
  printf("    Left-mouse click = set the center of zooming and rotating to picked surface point\n");
  printf("\n");
  printf("Camera control:\n");
  printf("    Left-button-drag = rotate the camera view \n");
  printf("    Middle-button-drag = zoom the camera view\n");
  printf("    Right-button-drag = pan the camera view\n");
  printf("    Scroll-wheel = zoom the camera view\n");
  printf("    Left-click = set the center of rotation and zooming\n");
  printf("    PAGE UP/DOWN = match the camera parameters to the next/prev image\n");
  printf("\n");
  printf("Clipbox control:\n");
  printf("    Up-arrow = move top of clipbox up\n");
  printf("    Down-arrow = move top of clipbox down\n");
  printf("    Right-arrow = move bottom of clipbox up\n");
  printf("    Left-arrow = move bottom of clipbox down\n");
  printf("    ESC = reset the clipbox (and remove all selections)\n");
  printf("\n");
  printf("Display options:\n");
  printf("    A = toggle display of Cartesian axes\n");
  printf("    B = toggle display of bounding boxes\n");
  printf("    C = toggle display of image viewpoints\n");
  printf("    E = toggle display of edges (wireframe)\n");
  printf("    F = toggle display of faces\n");
  printf("    H = print this list of commands\n");
  printf("    I = toggle display of images\n");
  printf("    M = toggle display of mesh\n");
  printf("    O = toggle display of objects\n");
  printf("    P = toggle display of panorama centers\n");
  printf("    R = toggle display of regions\n");
  printf("    S = toggle display of scene\n");
  printf("    V = toggle display of vertices\n");
  printf("    X = toggle display of clip box (only clips if visible)\n");
  printf("    SPACE = cycles through different colors schemes\n");
  printf("\n");
  printf("Query commands:\n");
  printf("    Left-button-click = select a region, object, or image\n");
  printf("    Left-button-double-click = print information about what is under cursor to stdout\n");
  printf("\n");
  printf(" Quit:\n");
  printf("    Ctrl-Q = quit \n");
  printf("\n");
}




////////////////////////////////////////////////////////////////////////
// Input functions
////////////////////////////////////////////////////////////////////////

static int
ReadHouse(char *filename)
{
  // Check filename
  if (!filename) return 1;

  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read house file 
  if (!house->ReadAsciiFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Read house from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Images = %d\n", house->images.NEntries());
    printf("  # Panoramas = %d\n", house->panoramas.NEntries());
    printf("  # Segments = %d\n", house->segments.NEntries());
    printf("  # Objects = %d\n", house->objects.NEntries());
    printf("  # Categories = %d\n", house->categories.NEntries());
    printf("  # Vertices = %d\n", house->vertices.NEntries());
    printf("  # Surfaces = %d\n", house->surfaces.NEntries());
    printf("  # Regions = %d\n", house->regions.NEntries());
    printf("  # Portals = %d\n", house->portals.NEntries());
    printf("  # Levels = %d\n", house->levels.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
ReadScene(char *filename)
{
  // Check filename
  if (!filename) return 1;
  if (!house) return 0;
  
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read house file 
  if (!house->ReadSceneFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    R3Scene *scene = house->scene;
    printf("Read scene from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Nodes = %d\n", scene->NNodes());
    printf("  # Lights = %d\n", scene->NLights());
    printf("  # Materials = %d\n", scene->NMaterials());
    printf("  # Brdfs = %d\n", scene->NBrdfs());
    printf("  # Textures = %d\n", scene->NTextures());
    printf("  # Referenced scenes = %d\n", scene->NReferencedScenes());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
ReadMesh(const char *filename)
{
  // Check filename
  if (!filename) return 1;
  if (!house) return 0;

  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read mesh
  if (!house->ReadMeshFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Read mesh ...\n");
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Faces = %d\n", house->mesh->NFaces());
    printf("  # Edges = %d\n", house->mesh->NEdges());
    printf("  # Vertices = %d\n", house->mesh->NVertices());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
ReadCategories(const char *filename)
{
  // Check stuff
  if (!filename) return 1;
  if (!house) return 0;
  
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read categories
  if (!house->ReadCategoryFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Read mturk categories from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Categories = %d\n", house->categories.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
ReadSegments(const char *filename)
{
  // Check stuff
  if (!filename) return 1;
  if (!house) return 0;
  
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read segments
  if (!house->ReadSegmentFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Read segments from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Segments = %d\n", house->segments.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
ReadObjects(const char *filename)
{
  // Check stuff
  if (!filename) return 1;
  if (!house) return 0;
  
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read objects
  if (!house->ReadObjectFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Read objects from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Objects = %d\n", house->objects.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



static int
ReadConfiguration(const char *filename) 
{
  // Check stuff
  if (!filename) return 1;
  if (!house) return 0;
  
  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read the configuration file
  if (!house->ReadConfigurationFile(filename)) return 0;
  
  // Print statistics
  if (print_verbose) {
    printf("Read configuration from %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Panorama = %d\n", house->panoramas.NEntries());
    printf("  # Images = %d\n", house->images.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



////////////////////////////////////////////////////////////////////////
// Output functions
////////////////////////////////////////////////////////////////////////

static int
WriteHouse(char *filename)
{
  // Check filename
  if (!filename) return 1;

  // Start statistics
  RNTime start_time;
  start_time.Read();

  // Read house file 
  if (!house->WriteAsciiFile(filename)) return 0;

  // Print statistics
  if (print_verbose) {
    printf("Wrote house to %s ...\n", filename);
    printf("  Time = %.2f seconds\n", start_time.Elapsed());
    printf("  # Images = %d\n", house->images.NEntries());
    printf("  # Panoramas = %d\n", house->panoramas.NEntries());
    printf("  # Segments = %d\n", house->segments.NEntries());
    printf("  # Objects = %d\n", house->objects.NEntries());
    printf("  # Categories = %d\n", house->categories.NEntries());
    printf("  # Vertices = %d\n", house->vertices.NEntries());
    printf("  # Surfaces = %d\n", house->surfaces.NEntries());
    printf("  # Regions = %d\n", house->regions.NEntries());
    printf("  # Portals = %d\n", house->portals.NEntries());
    printf("  # Levels = %d\n", house->levels.NEntries());
    fflush(stdout);
  }

  // Return success
  return 1;
}



////////////////////////////////////////////////////////////////////////
// Draw functions
////////////////////////////////////////////////////////////////////////

static void
DrawAxes(void)
{
  // Draw axes
  RNScalar d = house->bbox.DiagonalRadius();
  R3BeginLine();
  glColor3f(1, 0, 0);
  R3LoadPoint(R3zero_point + 0.5 * d * R3negx_vector);
  R3LoadPoint(R3zero_point + d * R3posx_vector);
  R3EndLine();
  R3BeginLine();
  glColor3f(0, 1, 0);
  R3LoadPoint(R3zero_point + 0.5 * d * R3negy_vector);
  R3LoadPoint(R3zero_point + d * R3posy_vector);
  R3EndLine();
  R3BeginLine();
  glColor3f(0, 0, 1);
  R3LoadPoint(R3zero_point + 0.5 * d * R3negz_vector);
  R3LoadPoint(R3zero_point + d * R3posz_vector);
  R3EndLine();
}



static void
LoadClipPlanes(void)
{
  // Check if clip planes are enabled
  if (show_clip_box) {
    // Load lo clip planes
    for (int dim = RN_X; dim <= RN_Z; dim++) {
      GLdouble plane_equation[4] = { 0, 0, 0, 0 };
      plane_equation[dim] = 1.0;
      plane_equation[3] = -(clip_box[RN_LO][dim] - 0.1);
      glClipPlane(GL_CLIP_PLANE0 + dim, plane_equation);
      glEnable(GL_CLIP_PLANE0 + dim);
    }

    // Load hi clip planes
    for (int dim = RN_X; dim <= RN_Z; dim++) {
      GLdouble plane_equation[4] = { 0, 0, 0, 0 };
      plane_equation[dim] = -1.0;
      plane_equation[3] = clip_box[RN_HI][dim] + 0.1;
      glClipPlane(GL_CLIP_PLANE0 + 3 + dim, plane_equation);
      glEnable(GL_CLIP_PLANE0 + 3 + dim);
    }
  }
  else {
    // Disable all clip planes
    for (int i = 0; i < 6; i++) {
      glDisable(GL_CLIP_PLANE0 + i);
    }
  }
}



////////////////////////////////////////////////////////////////////////
// Picking functions
////////////////////////////////////////////////////////////////////////

static void
SelectImage(MPImage *image)
{
  // Release old image
  if (selected_image) {
    selected_image->rgbd.ReleaseChannels();
    selected_image = NULL;
  }

  // Read new image
  if (image) {
    image->rgbd.ReadChannels();
    selected_image = image;
  }
}



static void
SnapImage(MPImage *image)
{
  // Select image
  SelectImage(image);

  // Set viewer stuff
  if (image) {
    viewer->RepositionCamera(image->rgbd.WorldViewpoint());
    viewer->ReorientCamera(image->rgbd.WorldTowards(), image->rgbd.WorldUp());
    center = image->rgbd.WorldViewpoint() + 3 * image->rgbd.WorldTowards();
  }

  // Remember snap image
  if (image) snap_image_index = image->house_index;
  snap_image = image;
}



static int
Pick(int x, int y,
  MPPanorama **hit_panorama = NULL, MPImage **hit_image = NULL,
  MPSegment **hit_segment = NULL, MPObject **hit_object = NULL, MPRegion **hit_region = NULL,
  R3SceneNode **hit_node = NULL, R3SceneElement **hit_element = NULL, R3Shape **hit_shape = NULL, R3MeshFace **hit_face = NULL,
  R3Point *hit_position = NULL, R3Vector *hit_normal = NULL, RNScalar *hit_t = NULL)
{
  // Initialize the result
  if (hit_panorama) *hit_panorama = NULL;
  if (hit_image) *hit_image = NULL;
  if (hit_segment) *hit_segment = NULL;
  if (hit_object) *hit_object = NULL;
  if (hit_region) *hit_region = NULL;
  if (hit_node) *hit_node = NULL;
  if (hit_element) *hit_element = NULL;
  if (hit_shape) *hit_shape = NULL;
  if (hit_face) *hit_face = NULL;
  if (hit_position) hit_position->Reset(0,0,0);
  if (hit_normal) hit_normal->Reset(0,0,0);
  if (hit_t) *hit_t = 0;
  
  // Clear window
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Set viewing transformation
  viewer->Camera().Load();
  LoadClipPlanes();

  // Set OpenGL stuff
  int pick_tolerance = 10;
  glLineWidth(pick_tolerance);
  glPointSize(pick_tolerance);
  glDisable(GL_LIGHTING);

  // Initialize bookkeeping
  RNArray<R3SceneNode *> nodes;
  RNArray<R3SceneElement *> elements;
  RNArray<R3Shape *> shapes;
  RNArray<R3Triangle *> triangles;

  // Draw scene with color indicating index
  if (house && house->scene && scene_draw_flags[MP_SHOW_SCENE] && scene_draw_flags[MP_DRAW_FACES]) {
    for (int i = 0; i < house->scene->NNodes(); i++) {
      R3SceneNode *node = house->scene->Node(i);
      if (node->NChildren() > 0) continue;
      for (int j = 0; j < node->NElements(); j++) {
        R3SceneElement *element = node->Element(j);
        for (int k = 0; k < element->NShapes(); k++) {
          R3Shape *shape = element->Shape(k);
          if (shape->ClassID() != R3TriangleArray::CLASS_ID()) continue;
          R3TriangleArray *triangle_array = (R3TriangleArray *) shape;
          for (int t = 0; t < triangle_array->NTriangles(); t++) {
            R3Triangle *triangle = triangle_array->Triangle(t);
            int index = triangles.NEntries() + 1;
            unsigned char rgba[4];
            rgba[0] = (index >> 16) & 0xFF;
            rgba[1] = (index >> 8) & 0xFF;
            rgba[2] = index & 0xFF;
            rgba[3] = MP_SCENE_TAG;
            glColor4ubv(rgba);
            triangle->Draw(R3_SURFACES_DRAW_FLAG);
            triangles.Insert(triangle);
            shapes.Insert(shape);
            elements.Insert(element);
            nodes.Insert(node);
          }
        }
      }
    }
  }

  // Draw image with color indicating index
  if (image_draw_flags[MP_SHOW_IMAGES]) house->DrawImages(MP_SHOW_IMAGES | MP_DRAW_DEPICTIONS | MP_COLOR_FOR_PICK);
  if (panorama_draw_flags[MP_SHOW_PANORAMAS]) house->DrawPanoramas(panorama_draw_flags | MP_COLOR_FOR_PICK);
  if (object_draw_flags[MP_SHOW_OBJECTS]) house->DrawObjects(object_draw_flags | MP_COLOR_FOR_PICK);
  if (region_draw_flags[MP_SHOW_REGIONS]) house->DrawRegions(region_draw_flags | MP_COLOR_FOR_PICK);
  if (mesh_draw_flags[MP_SHOW_MESH]) house->DrawMesh(mesh_draw_flags | MP_COLOR_FOR_PICK);
    
  // Reset OpenGL stuff
  glLineWidth(1);
  glPointSize(1);
  glFinish();

  // Read color buffer at cursor position
  unsigned char rgba[4];
  glReadPixels(x, y, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
  if ((rgba[0] == 0) && (rgba[1] == 0) && (rgba[2] == 0)) return 0;

  // Parse hit 
  int r = rgba[0] & 0xFF;
  int g = rgba[1] & 0xFF;
  int b = rgba[2] & 0xFF;
  int a = rgba[3] & 0xFF;
  int index = ((r << 16) | (g << 8) | b) - 1;
  if (a == MP_SCENE_TAG) {
    // Hit triangle of scene
    if ((index < 0) || (index >= triangles.NEntries())) return 0;
    if (hit_node) *hit_node = nodes.Kth(index);
    if (hit_element) *hit_element = elements.Kth(index);
    if (hit_shape) *hit_shape = shapes.Kth(index);
    if (hit_position || hit_normal || hit_t) {
      R3Triangle *triangle = triangles.Kth(index);
      if (hit_normal) *hit_normal = triangle->Normal();
      R3Ray ray = viewer->WorldRay(x, y);
      if (!R3Intersects(ray, triangle->Plane(), hit_position, hit_t)) {
        printf("Unable to find ray intersection\n");
        return 0;
      }
    }
  }
  else if (a == MP_MESH_TAG) {
    // Hit face of mesh
    if ((index < 0) || (index >= house->mesh->NFaces())) return 0;
    R3MeshFace *face = house->mesh->Face(index);
    if (hit_normal) *hit_normal = house->mesh->FaceNormal(face);
    R3Ray ray = viewer->WorldRay(x, y);
    if (hit_face) *hit_face = face;
    if (hit_position) *hit_position = house->mesh->FaceCentroid(face);
    if (hit_t) *hit_t = ray.T(house->mesh->FaceCentroid(face));
    R3MeshIntersection intersection;
    if (house->mesh->Intersection(ray, face, &intersection)) {
      if (hit_position) *hit_position = intersection.point;
      if (hit_t) *hit_t = intersection.t;
    }
  }
  else if ((a == MP_IMAGE_TAG) && house) {
    // Hit image
    if ((index < 0) || (index >= house->images.NEntries())) return 0;
    if (hit_image) *hit_image = house->images.Kth(index);
    if (hit_position) *hit_position = house->images.Kth(index)->position;
  }
  else if ((a == MP_PANORAMA_TAG) && house) {
    // Hit panorama
    if ((index < 0) || (index >= house->panoramas.NEntries())) return 0;
    if (hit_panorama) *hit_panorama = house->panoramas.Kth(index);
    if (hit_position) *hit_position = house->panoramas.Kth(index)->position;
  }
  else if ((a == MP_SEGMENT_TAG) && house) {
    // Hit segment
    if ((index < 0) || (index >= house->segments.NEntries())) return 0;
    if (hit_segment) *hit_segment = house->segments.Kth(index);
  }
  else if ((a == MP_OBJECT_TAG) && house) {
    // Hit object
    if ((index < 0) || (index >= house->objects.NEntries())) return 0;
    if (hit_object) *hit_object = house->objects.Kth(index);
  }
  else if ((a == MP_SURFACE_TAG) && house) {
    // Hit surface
    if ((index < 0) || (index >= house->surfaces.NEntries())) return 0;
    R3Ray ray = viewer->WorldRay(x, y);
    R3Plane plane(house->surfaces.Kth(index)->position, house->surfaces.Kth(index)->normal);
    MPSurface *surface = house->surfaces.Kth(index);
    if (hit_region) *hit_region = surface->region;
    if (hit_position || hit_t) R3Intersects(ray, plane, hit_position, hit_t);
    if (hit_normal) *hit_normal = house->surfaces.Kth(index)->normal;
  }
  else if ((a == MP_REGION_TAG) && house) {
    // Hit region
    if ((index < 0) || (index >= house->regions.NEntries())) return 0;
    if (hit_region) *hit_region = house->regions.Kth(index);
    if (hit_position) *hit_position = house->regions.Kth(index)->position;
  }
     
  // Return hit position
  if (hit_position && hit_position->IsZero()) {
    GLfloat depth;
    GLdouble p[3];
    GLint viewport[4];
    GLdouble modelview_matrix[16];
    GLdouble projection_matrix[16];
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
    glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    gluUnProject(x, y, depth, modelview_matrix, projection_matrix, viewport, &(p[0]), &(p[1]), &(p[2]));
    R3Point position(p[0], p[1], p[2]);
    *hit_position = position;
  }
  
  // Return success
  return 1;
}



////////////////////////////////////////////////////////////////////////
// GLUT user interface functions
////////////////////////////////////////////////////////////////////////

void GLUTStop(void)
{
  // Destroy window 
  glutDestroyWindow(GLUTwindow);

  // Exit
  exit(0);
}



void Draw(GLbitfield clearFlags = GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
{
    // Clear window 
    glClearColor(background.R(), background.G(), background.B(), 1.0);
    glClear(clearFlags);

    // Set backface culling
    if (show_backfacing) glDisable(GL_CULL_FACE);
    else glEnable(GL_CULL_FACE);

    // Set viewing transformation
    if (snap_image) {
        // Set viewport
        glViewport(0, 0, snap_image->width / 2, snap_image->height / 2);

        // Set perspective transformation
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        snap_image->rgbd.ProjectionMatrix().Load();

        // Set modelview transformation
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        snap_image->extrinsics.Load();
    }
    else {
        viewer->Load();
    }

    // Set lights
    static GLfloat light1_position[] = { 3.0, 4.0, 5.0, 0.0 };
    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
    static GLfloat light2_position[] = { -3.0, -2.0, -3.0, 0.0 };
    glLightfv(GL_LIGHT2, GL_POSITION, light2_position);

    // Set clip planes
    LoadClipPlanes();

    // Draw selected image
    if (selected_image) {
        glDisable(GL_LIGHTING);
        glColor3d(1.0, 1.0, 0.0);
        glLineWidth(3.0);
        selected_image->DrawCamera();
        glLineWidth(1.0);
        selected_image->Draw(MP_SHOW_IMAGES | image_draw_flags | MP_COLOR_BY_RGB);
    }

    // Draw selected region
    if (selected_region) {
        glDisable(GL_LIGHTING);
        glColor3d(1.0, 1.0, 0.0);
        glLineWidth(3.0);
        selected_region->DrawSurfaces(MP_SHOW_SURFACES | MP_DRAW_EDGES);
        glLineWidth(1.0);
    }

    // Draw selected object
    if (selected_object) {
        glDisable(GL_LIGHTING);
        glColor3d(1.0, 1.0, 0.0);
        glLineWidth(3.0);
        selected_object->DrawBBox(MP_DRAW_EDGES);
        glLineWidth(1.0);
    }

    // Draw house elements
    glColor3d(0.5, 0.5, 0.5);
    if (image_draw_flags[MP_SHOW_IMAGES]) house->DrawImages(MP_SHOW_IMAGES | MP_DRAW_DEPICTIONS | color_scheme);
    if (panorama_draw_flags[MP_SHOW_PANORAMAS]) house->DrawPanoramas(panorama_draw_flags | color_scheme);
    if (region_draw_flags[MP_SHOW_REGIONS]) house->DrawRegions(region_draw_flags | color_scheme);
    if (level_draw_flags[MP_SHOW_LEVELS]) house->DrawLevels(level_draw_flags | color_scheme);
    if (object_draw_flags[MP_SHOW_OBJECTS]) house->DrawObjects(object_draw_flags | color_scheme);
    if (scene_draw_flags[MP_SHOW_SCENE]) house->DrawScene(scene_draw_flags | color_scheme);
    if (mesh_draw_flags[MP_SHOW_MESH]) house->DrawMesh(mesh_draw_flags | color_scheme);

    // Draw clip box
    if (show_clip_box) {
        glDisable(GL_LIGHTING);
        glColor3d(0.4, 0.4, 0.4);
        clip_box.Outline();
    }

    // Draw axes
    if (show_axes) {
        glDisable(GL_LIGHTING);
        glLineWidth(3);
        DrawAxes();
        glLineWidth(1);
    }

    // Capture image and exit
    if (output_image_filename) {
        R2Image image(GLUTwindow_width, GLUTwindow_height, 3);
        image.Capture();
        image.Write(output_image_filename);
        if (batch) GLUTStop();
    }

}
void GLUTRedraw(void)
{
    Draw();
  // Swap buffers 
  glutSwapBuffers();
}    



void GLUTResize(int w, int h)
{
  // Resize window
  glViewport(0, 0, w, h);

  // Resize viewer viewport
  viewer->ResizeViewport(0, 0, w, h);

  // Remember window size 
  GLUTwindow_width = w;
  GLUTwindow_height = h;

  // Redraw
  glutPostRedisplay();
}



void GLUTMotion(int x, int y)
{
  // Invert y coordinate
  y = GLUTwindow_height - y;

  // Compute mouse movement
  int dx = x - GLUTmouse[0];
  int dy = y - GLUTmouse[1];
  
  // Update mouse drag
  GLUTmouse_drag += dx*dx + dy*dy;

  // Process mouse movement command
  if (GLUTmodifiers & GLUT_ACTIVE_SHIFT) {
    // Camera in hand navigation
    const R3Camera& camera = viewer->Camera();
    if (GLUTbutton[0]) viewer->RotateCamera(camera.Right(), -0.001*dy);
    if (GLUTbutton[0]) viewer->RotateCamera(R3posy_vector, 0.001*dx);
    else if (GLUTbutton[1]) viewer->TranslateCamera((0.01*dx + 0.01*dy)*camera.Towards());
    else if (GLUTbutton[2]) viewer->TranslateCamera((0.01*dx)*camera.Right() + (0.01*dy)*camera.Down());
    if (GLUTbutton[0] || GLUTbutton[1] || GLUTbutton[2]) glutPostRedisplay();
  }
  else {
    // World in hand navigation
    if (GLUTbutton[0]) viewer->RotateWorld(1.0, center, x, y, dx, dy);
    else if (GLUTbutton[1]) viewer->ScaleWorld(1.0, center, x, y, dx, dy);
    else if (GLUTbutton[2]) viewer->TranslateWorld(1.0, center, x, y, dx, dy);
    if (GLUTbutton[0] || GLUTbutton[1] || GLUTbutton[2]) glutPostRedisplay();
  }

  // Remember mouse position 
  GLUTmouse[0] = x;
  GLUTmouse[1] = y;
}



void GLUTMouse(int button, int state, int x, int y)
{
  // Invert y coordinate
  y = GLUTwindow_height - y;

  // Mouse is going down
  if (state == GLUT_DOWN) {
    // Reset mouse state
    GLUTmouse_drag = 0;
    snap_image = NULL;
  }
  else {
    // Process thumbwheel
    if (button == 3) viewer->ScaleWorld(center, 0.9);
    else if (button == 4) viewer->ScaleWorld(center, 1.1);

    // Process button clicks
    if (button == 0) {
      // Check for double click  
      static RNBoolean double_click = FALSE;
      static RNTime last_mouse_up_time;
      double_click = (!double_click) && (last_mouse_up_time.Elapsed() < 0.4);
      last_mouse_up_time.Read();

      // Check for click (rather than drag)
      if (GLUTmouse_drag < 100) {
        // Find whatever was below cursor
        MPPanorama *panorama = NULL;
        MPImage *image = NULL;
        MPSegment *segment = NULL;
        MPObject *object = NULL;
        MPRegion *region = NULL;
        R3SceneNode *node = NULL;
        R3MeshFace *face = NULL;
        R3Point position;
        R3Vector normal;
        if (Pick(x, y, &panorama, &image, &segment, &object, &region, &node, NULL, NULL, &face, &position, &normal)) {
          // Check for double click
          if (double_click) {
            // Print info about whatever was picked
            if (panorama) printf("Panorama %d at ", panorama->house_index);
            else if (image) printf("Image %d at ", image->house_index);
            else if (segment) printf("Segment %d at ", segment->house_index);
            else if (object) printf("Object %d at ", object->house_index);
            else if (region) printf("Region %d", region->house_index);
            else if (node) printf("Node %d", node->SceneIndex());
            else if (face) printf("Face %d", house->mesh->FaceID(face));
            printf(" at position %g %g %g", position.X(), position.Y(), position.Z());
            printf("\n");
          }
        }

        // Set selection
        selected_region = region;
        selected_object = object;
        SelectImage(image);
        
        // Set viewing center point
        center = position;
      }
    }
  }
  
  // Remember button state 
  int b = (button == GLUT_LEFT_BUTTON) ? 0 : ((button == GLUT_MIDDLE_BUTTON) ? 1 : 2);
  GLUTbutton[b] = (state == GLUT_DOWN) ? 1 : 0;

  // Remember modifiers 
  GLUTmodifiers = glutGetModifiers();

   // Remember mouse position 
  GLUTmouse[0] = x;
  GLUTmouse[1] = y;

  // Redraw
  glutPostRedisplay();
}



void GLUTSpecial(int key, int x, int y)
{
  // Invert y coordinate
  y = GLUTwindow_height - y;

  // Process keyboard button event
  switch (key) {
  case GLUT_KEY_F1: {
    MPImage *image = NULL;
    if (Pick(x, y, NULL, &image)) {
      SnapImage(image);
    }
    break; }
    
  case GLUT_KEY_PAGE_UP:
    if (house->images.NEntries() > 0) {
      if (++snap_image_index >= house->images.NEntries()) snap_image_index = house->images.NEntries()-1;
      SnapImage(house->images.Kth(snap_image_index));
      printf("Image %d\n", snap_image_index);
    }
    break;
    
  case GLUT_KEY_PAGE_DOWN:
    if (house->images.NEntries() > 0) {
      if (--snap_image_index < 0) snap_image_index = 0;
      SnapImage(house->images.Kth(snap_image_index));
      printf("Image %d\n", snap_image_index);
    }
    break;
    
  case GLUT_KEY_LEFT:
    clip_box[0][2] -= 0.1;
    if (clip_box[0][2] > clip_box[1][2]) clip_box[0][2] = clip_box[1][2];
    break;

  case GLUT_KEY_RIGHT:
    clip_box[0][2] += 0.1;
    if (clip_box[0][2] < house->bbox.ZMin()) clip_box[0][2] = house->bbox.ZMin();
    break;

  case GLUT_KEY_DOWN:
    clip_box[1][2] -= 0.1;
    if (clip_box[1][2] < clip_box[0][2]) clip_box[1][2] = clip_box[0][2];
    break;

  case GLUT_KEY_UP:
    clip_box[1][2] += 0.1;
    if (clip_box[1][2] > house->bbox.ZMax()) clip_box[1][2] = house->bbox.ZMax();
    break;
  }

  // Remember mouse position 
  GLUTmouse[0] = x;
  GLUTmouse[1] = y;

  // Remember modifiers 
  GLUTmodifiers = glutGetModifiers();

  // Redraw
  glutPostRedisplay();
}



void GLUTKeyboard(unsigned char key, int x, int y)
{
  // Invert y coordinate
  y = GLUTwindow_height - y;

  // Process keyboard button event
  switch (key) {
  case 'A': 
  case 'a':
    show_axes = !show_axes;
    break;

  case 'B': 
  case 'b':
    region_draw_flags.XOR(MP_DRAW_BBOXES);
    object_draw_flags.XOR(MP_DRAW_BBOXES);
    image_draw_flags.XOR(MP_DRAW_BBOXES);
    break;

  case 'C': 
  case 'c':
    image_draw_flags.XOR(MP_SHOW_IMAGES);
    break;
    
  case 'E': 
  case 'e': 
    region_draw_flags.XOR(MP_DRAW_EDGES);
    object_draw_flags.XOR(MP_DRAW_EDGES);
    scene_draw_flags.XOR(MP_DRAW_EDGES);
    mesh_draw_flags.XOR(MP_DRAW_EDGES);
    break;

  case 'F': 
  case 'f': 
    region_draw_flags.XOR(MP_DRAW_FACES);
    object_draw_flags.XOR(MP_DRAW_FACES);
    //scene_draw_flags.XOR(MP_DRAW_FACES);
    mesh_draw_flags.XOR(MP_DRAW_FACES);
    break;

  case 'H':
  case 'h':
    PrintCommands();
    break; 

  case 'I': 
  case 'i':
    image_draw_flags.XOR(MP_DRAW_IMAGES);
    break;

  case 'L': 
  case 'l':
    region_draw_flags.XOR(MP_DRAW_LABELS);
    object_draw_flags.XOR(MP_DRAW_LABELS);
    break;

  case 'M': 
  case 'm':
    mesh_draw_flags.XOR(MP_SHOW_MESH);
    break;

  case 'O': 
  case 'o':
    object_draw_flags.XOR(MP_SHOW_OBJECTS | MP_SHOW_SEGMENTS);
    break;

  case 'P': 
  case 'p':
    panorama_draw_flags.XOR(MP_SHOW_PANORAMAS);
    break;

  case 'R': 
  case 'r':
    region_draw_flags.XOR(MP_SHOW_REGIONS | MP_SHOW_SURFACES);
    break;

  case 'S': 
  case 's':
    scene_draw_flags.XOR(MP_SHOW_SCENE);
    break;

  case 'V': 
  case 'v':
    //region_draw_flags.XOR(MP_DRAW_VERTICES);
    //image_draw_flags.XOR(MP_DRAW_VERTICES);
    //object_draw_flags.XOR(MP_DRAW_VERTICES);
    mesh_draw_flags.XOR(MP_DRAW_VERTICES);
    break;

  case 'X':
  case 'x':
    show_clip_box = !show_clip_box;
    break;

  case 'Q':
  case 'q':
      //region_draw_flags.XOR(MP_COLOR_FOR_PICK);
      //object_draw_flags.XOR(MP_COLOR_FOR_PICK);
      //mesh_draw_flags.XOR(MP_COLOR_FOR_PICK);
      MP_USE_LIGHTING = !MP_USE_LIGHTING;
      break;

  case ' ':
    if (color_scheme == MP_COLOR_BY_RGB)
      color_scheme = MP_COLOR_BY_OBJECT | MP_COLOR_BY_LABEL;
    else if (color_scheme == (MP_COLOR_BY_OBJECT | MP_COLOR_BY_LABEL))
      color_scheme = MP_COLOR_BY_OBJECT | MP_COLOR_BY_INDEX;
    else if (color_scheme == (MP_COLOR_BY_OBJECT | MP_COLOR_BY_INDEX))
      color_scheme = MP_COLOR_BY_REGION | MP_COLOR_BY_LABEL;
    else if (color_scheme == (MP_COLOR_BY_REGION | MP_COLOR_BY_LABEL))
      color_scheme = MP_COLOR_BY_REGION | MP_COLOR_BY_INDEX;
    else if (color_scheme == (MP_COLOR_BY_REGION | MP_COLOR_BY_INDEX))
      color_scheme = MP_COLOR_BY_LEVEL | MP_COLOR_BY_INDEX;
    else if (color_scheme == (MP_COLOR_BY_LEVEL | MP_COLOR_BY_INDEX))
      color_scheme = MP_COLOR_BY_RGB;
    break;

  case '1':
      SnapImage(house->images.Kth(1));
      printf("Jump to 1\n");
      break;
  case '2':
      SnapImage(house->images.Kth(551));
      printf("Jump to 551\n");
      break;
  case '3':
      SnapImage(house->images.Kth(542));
      printf("Jump to 542\n");
      break;
  case '4':
      SnapImage(house->images.Kth(660));
      printf("Jump to 660\n");
      break;
  case '5':
      SnapImage(house->images.Kth(594));
      printf("Jump to 594\n");
      break;

  case 27: // ESC
    // Reset the selections and clipbox
    clip_box = house->bbox;
    selected_region = NULL;
    selected_object = NULL;
    selected_image = NULL;
    snap_image = NULL;
    break;

  case 17: // ctrl-Q
    GLUTStop();
    break;
  }
  
  // Remember mouse position 
  GLUTmouse[0] = x;
  GLUTmouse[1] = y;

  // Remember modifiers 
  GLUTmodifiers = glutGetModifiers();

  // Redraw
  glutPostRedisplay();  
}




void GLUTInit(int *argc, char **argv)
{
  // Open window 
  glutInit(argc, argv);
  glutInitWindowPosition(100, 100);
  glutInitWindowSize(GLUTwindow_width, GLUTwindow_height);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA);
  GLUTwindow = glutCreateWindow("House Viewer");

  // Initialize lighting
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  static GLfloat lmodel_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
  glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  static GLfloat light0_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
  static GLfloat light0_position[] = { 0.0, 0.0, 1.0, 0.0 };
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
  glEnable(GL_LIGHT0);
  static GLfloat light1_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
  glEnable(GL_LIGHT1);
  static GLfloat light2_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
  glLightfv(GL_LIGHT2, GL_DIFFUSE, light2_diffuse);
  glEnable(GL_LIGHT2);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING); 

  // Initialize color settings
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  // Initialize depth testing
  glEnable(GL_DEPTH_TEST);

  // Initialize GLUT callback functions 
  glutDisplayFunc(GLUTRedraw);
  glutReshapeFunc(GLUTResize);
  glutKeyboardFunc(GLUTKeyboard);
  glutSpecialFunc(GLUTSpecial);
  glutMouseFunc(GLUTMouse);
  glutMotionFunc(GLUTMotion);
}



void GLUTMainLoop(void)
{
  // Just checking
  if (house->bbox.IsEmpty()) return;
  
  // Initialize viewing stuff
  center = house->bbox.Centroid();
  clip_box = house->bbox;

  // Setup camera view looking down the Z axis
  assert(!house->bbox.IsEmpty());
  RNLength r = house->bbox.DiagonalRadius();
  assert((r > 0.0) && RNIsFinite(r));
  if (!initial_camera) initial_camera_origin = house->bbox.Centroid() - initial_camera_towards * (2.5 * r);
  R3Camera camera(initial_camera_origin, initial_camera_towards, initial_camera_up, 0.54, 0.45, 0.01 * r, 100.0 * r);
  R2Viewport viewport(0, 0, GLUTwindow_width, GLUTwindow_height);
  viewer = new R3Viewer(camera, viewport);
  
  // Run main loop -- never returns 
  glutMainLoop();
}


 
////////////////////////////////////////////////////////////////////////
// Argument parsing functions
////////////////////////////////////////////////////////////////////////

static int 
ParseArgs(int argc, char **argv)
{
  // Remember if an output was specified
  RNBoolean input = FALSE;
  
  // Parse arguments
  argc--; argv++;
  while (argc > 0) {
    if ((*argv)[0] == '-') {
      if (!strcmp(*argv, "-v")) print_verbose = 1;
      else if (!strcmp(*argv, "-batch")) batch = 1;
      else if (!strcmp(*argv, "-output_house")) { argc--; argv++; output_house_filename = *argv; }
      else if (!strcmp(*argv, "-output_image")) { argc--; argv++; output_image_filename = *argv; }
      else if (!strcmp(*argv, "-input_house")) { argc--; argv++; input_house_filename = *argv; input = TRUE; }
      else if (!strcmp(*argv, "-input_scene")) { argc--; argv++; input_scene_filename = *argv; input = TRUE; }
      else if (!strcmp(*argv, "-input_mesh")) { argc--; argv++; input_mesh_filename = *argv; input = TRUE; }
      else if (!strcmp(*argv, "-input_categories")) { argc--; argv++; input_categories_filename = *argv; }
      else if (!strcmp(*argv, "-input_segments")) { argc--; argv++; input_segments_filename = *argv; }
      else if (!strcmp(*argv, "-input_objects")) { argc--; argv++; input_objects_filename = *argv; }
      else if (!strcmp(*argv, "-input_configuration")) { argc--; argv++; input_configuration_filename = *argv; input = TRUE; }
      else if (!strcmp(*argv, "-write_images")) { argc--; argv++; output_images_path = *argv; }
      else if (!strcmp(*argv, "-match_meshes")) { argc--; argv++; match_meshes = *argv; }
      else if (!strcmp(*argv, "-background")) {
        argv++; argc--; background[0] = atof(*argv);
        argv++; argc--; background[1] = atof(*argv);
        argv++; argc--; background[2] = atof(*argv);
      }
      else if (!strcmp(*argv, "-window")) { 
        argv++; argc--; GLUTwindow_width = atoi(*argv); 
        argv++; argc--; GLUTwindow_height = atoi(*argv); 
      }
      else if (!strcmp(*argv, "-camera")) {
        RNCoord x, y, z, tx, ty, tz, ux, uy, uz;
        argv++; argc--; x = atof(*argv);
        argv++; argc--; y = atof(*argv);
        argv++; argc--; z = atof(*argv);
        argv++; argc--; tx = atof(*argv);
        argv++; argc--; ty = atof(*argv);
        argv++; argc--; tz = atof(*argv);
        argv++; argc--; ux = atof(*argv);
        argv++; argc--; uy = atof(*argv);
        argv++; argc--; uz = atof(*argv);
        initial_camera_origin = R3Point(x, y, z);
        initial_camera_towards.Reset(tx, ty, tz);
        initial_camera_up.Reset(ux, uy, uz);
        initial_camera = TRUE;
      }
      else {
        fprintf(stderr, "Invalid program argument: %s", *argv);
        exit(1);
      }
      argv++; argc--;
    }
    else {
      if (!input_house_filename && strstr(*argv, ".house")) { input_house_filename = *argv; input = TRUE; }
      else if (!input_scene_filename && strstr(*argv, ".obj")) { input_scene_filename = *argv; input = TRUE; }
      else if (!input_mesh_filename && strstr(*argv, ".ply")) { input_mesh_filename = *argv; input = TRUE; }
      else if (!input_categories_filename && strstr(*argv, ".tsv")) { input_categories_filename = *argv; }
      else if (!input_segments_filename && strstr(*argv, ".fsegs.json")) { input_segments_filename = *argv; }
      else if (!input_objects_filename && strstr(*argv, ".semseg.json")) { input_objects_filename = *argv; }
      else if (!input_ssa_filename && strstr(*argv, ".ssa")) { input_ssa_filename = *argv; input = TRUE; }
      else if (!input_ssb_filename && strstr(*argv, ".ssb")) { input_ssb_filename = *argv; input = TRUE; }
      else if (!input_configuration_filename && strstr(*argv, ".conf")) { input_configuration_filename = *argv; input = TRUE; }
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
    }
  }

  // Check filenames
  if (!input) {
    PrintUsage();
    return 0;
  }

  // Return OK status 
  return 1;
}


////////////////////////////////////////////////////////////////////////
// Main function
////////////////////////////////////////////////////////////////////////

void batchWriteImages()
{
    std::string output_path(output_images_path);
    printf("Write output images to %s\n", output_images_path);
    if (*output_path.rbegin() != '/' || *output_path.rbegin() != '\\') {
        output_path += "\\";
    }

    //init from GLUTMainLoop
    if (house->bbox.IsEmpty()) return;

    // Initialize viewing stuff
    center = house->bbox.Centroid();
    clip_box = house->bbox;

    // Setup camera view looking down the Z axis
    assert(!house->bbox.IsEmpty());
    RNLength r = house->bbox.DiagonalRadius();
    assert((r > 0.0) && RNIsFinite(r));
    if (!initial_camera) initial_camera_origin = house->bbox.Centroid() - initial_camera_towards * (2.5 * r);
    R3Camera camera(initial_camera_origin, initial_camera_towards, initial_camera_up, 0.54, 0.45, 0.01 * r, 100.0 * r);
    R2Viewport viewport(0, 0, GLUTwindow_width, GLUTwindow_height);
    viewer = new R3Viewer(camera, viewport);
	
    //prepare settings
    color_scheme = MP_COLOR_BY_OBJECT | MP_COLOR_BY_LABEL;
    const auto old_region_draw_flags = static_cast<unsigned long>(region_draw_flags);
    const auto old_object_draw_flags = static_cast<unsigned long>(object_draw_flags);
    const auto old_mesh_draw_flags = static_cast<unsigned long>(mesh_draw_flags);
    region_draw_flags.Reset(0);
    object_draw_flags.Reset(0);
    mesh_draw_flags.Reset(0);
    MP_USE_LIGHTING = false;
    show_clip_box = false;
    show_axes = false;

    GLint viewport2[4];
    glGetIntegerv(GL_VIEWPORT, viewport2);
    R2Image image(viewport2[2], viewport2[3], 3);
    std::vector<char> output_filename(output_path.length() + 50);

    //create category information
    struct CategoryData
    {
        int index;
        RNRgb color;
        const char* name;
    };
    int maxIndex = 0;
    std::map<int, CategoryData> indexToCategory;
    std::vector<CategoryData> categories;
    indexToCategory[0] = CategoryData{ 0, {0,0,0}, "unlabeled" };
    for (int i = 0; i < house->categories.NEntries(); ++i)
    {
        int index = house->categories[i]->mpcat40_id;
        RNRgb color = GetColor(index);
        const char* name = house->categories[i]->mpcat40_name;
        if (indexToCategory.count(index) == 0) {
            indexToCategory[index] = { index, color, name };
            categories.push_back(CategoryData{ index, color, name });
        }
        maxIndex = max(maxIndex, index);
    }

    //write category information
    sprintf_s(&output_filename[0], output_filename.size(),
        "%scategories.tsv", output_path.c_str());
    FILE* cat_file = fopen(output_filename.data(), "w");
	if (cat_file == nullptr)
	{
        printf("Unable to write category file! Does the output folder exist?\n");
        return;
	}
    fprintf(cat_file, "Index\tcat40-name\tcolor\n");
    for (int i = 0; i <= maxIndex; ++i) {
        const auto& d = indexToCategory[i];
        int r = max(0, min(255, int(round(d.color.R() * 255))));
        int g = max(0, min(255, int(round(d.color.G() * 255))));
        int b = max(0, min(255, int(round(d.color.B() * 255))));
        fprintf(cat_file, "%d\t%s\t#%02x%02x%02x\n",
            d.index, d.name, r, g, b);
    }
    fclose(cat_file);
	
	//loop over images
	for (snap_image_index=0; snap_image_index<house->images.NEntries(); ++snap_image_index)
	{
        printf("\r%3d / %3d", (snap_image_index + 1), house->images.NEntries());
        fflush(stdout);
        SnapImage(house->images.Kth(snap_image_index));

		//render real image
        image_draw_flags.Add(MP_DRAW_IMAGES);
        Draw();
        image.Capture();
        sprintf_s(&output_filename[0], output_filename.size(),
            "%sreal_%05d.png", output_path.c_str(), snap_image_index);
        image.Write(output_filename.data());
        image_draw_flags.Remove(MP_DRAW_IMAGES);

		//render virtual image
        scene_draw_flags.Add(MP_SHOW_SCENE);
        Draw();
        image.Capture();
        sprintf_s(&output_filename[0], output_filename.size(),
            "%ssynth_%05d.png", output_path.c_str(), snap_image_index);
        image.Write(output_filename.data());
        scene_draw_flags.Remove(MP_SHOW_SCENE);

		//render segmentation
        object_draw_flags.Reset(old_object_draw_flags);
        object_draw_flags.Add(MP_SHOW_OBJECTS | MP_DRAW_FACES); // | MP_DRAW_FACES);
        //region_draw_flags.Add(MP_SHOW_REGIONS | MP_DRAW_FACES | MP_DRAW_DEPICTIONS);
        //object_draw_flags.Add(MP_SHOW_OBJECTS | MP_DRAW_FACES);
        glPolygonOffset(-2.0f, -5);
        glEnable(GL_POLYGON_OFFSET_FILL);
        Draw(GL_COLOR_BUFFER_BIT);
        glDisable(GL_POLYGON_OFFSET_FILL);
        image.Capture();
        sprintf_s(&output_filename[0], output_filename.size(),
            "%sclasses_%05d.png", output_path.c_str(), snap_image_index);
#if 1 // no palette
        image.Write(output_filename.data());
#else // with palette
        {
            unsigned paletteSize = maxIndex + 1;
            assert(paletteSize <= PNG_MAX_PALETTE_LENGTH);
            FILE* fp = fopen(output_filename.data(), "wb");
            png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
            png_infop info_ptr = png_create_info_struct(png_ptr);
            png_set_IHDR(png_ptr, info_ptr, image.Width(), image.Height(),
                8, PNG_COLOR_TYPE_PALETTE, PNG_INTERLACE_NONE,
                PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);
            png_color* palette = (png_color*)png_malloc(png_ptr, paletteSize * sizeof(png_color));
            for (unsigned p = 0; p < paletteSize; p++)
            {
                png_color* col = &palette[p];
                const auto& d = indexToCategory[p];
                int r = max(0, min(255, int(round(d.color.R() * 255))));
                int g = max(0, min(255, int(round(d.color.G() * 255))));
                int b = max(0, min(255, int(round(d.color.B() * 255))));
                col->red = r;
                col->green = g;
                col->blue = b;
            }
            png_set_PLTE(png_ptr, info_ptr, palette, paletteSize);
            png_init_io(png_ptr, fp);
            png_write_info(png_ptr, info_ptr);
            std::vector<unsigned char> pixels(image.Width()* image.Height(), 0);
            int matchingErrors = 0;
        	for (int y=0; y<image.Height(); ++y)
                for (int x=0; x<image.Width(); ++x)
                {
                    auto col1 = image.PixelRGB(x, y);
                	//search for matching color
                    static double EPSILON = 2.0 / 255.0;
                    int index = -1;
                	for (const auto& cat : categories)
                	{
                		if (abs(cat.color.R() - col1.R())<=EPSILON &&
                            abs(cat.color.G() - col1.G()) <= EPSILON &&
                            abs(cat.color.B() - col1.B()) <= EPSILON)
                		{
                            index = cat.index;
                            break;
                		}
                	}
                	if (index==-1)
                	{
                        matchingErrors++;
                        index = 0;
                	}
                    pixels[x + image.Width() * y] = static_cast<unsigned char>(index);
                }
            png_bytep* row_pointers = (png_bytep*)png_malloc(png_ptr, image.Height() * png_sizeof(png_bytep));
            for (int i = 0; i < image.Height(); i++) 
                row_pointers[i] = &pixels[(image.Height() - i - 1) * image.Width()];
            png_write_image(png_ptr, row_pointers);
            png_write_end(png_ptr, info_ptr);
            png_free(png_ptr, palette);
            png_destroy_write_struct(&png_ptr, &info_ptr);
            fclose(fp);
        }
#endif
		
        region_draw_flags.Reset(0);
        object_draw_flags.Reset(0);
        mesh_draw_flags.Reset(0);
	}
    printf("\n");
}


////////////////////////////////////////////////////////////////////////
// Match "scene" and "mesh" meshes.
// Write the category ids from "mesh" into the high-resolution mesh from "scene"
// and save as ply
////////////////////////////////////////////////////////////////////////
template<typename T>
std::tuple<T, T, T> sort3(T a, T b, T c)
{
    if (a < b && a < c) return { a, min(b, c), max(b, c) };
    if (b < a && b < c) return { b, min(a, c), max(a, c) };
    else return { c, min(a, b), max(a, b) };
}
namespace std { //https://stackoverflow.com/a/21439212/1786598
    namespace
    {

        // Code from boost
        // Reciprocal of the golden ratio helps spread entropy
        //     and handles duplicates.
        // See Mike Seymour in magic-numbers-in-boosthash-combine:
        //     https://stackoverflow.com/questions/4948780

        template <class T>
        inline void hash_combine(std::size_t& seed, T const& v)
        {
            seed ^= hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }

        // Recursive template code derived from Matthieu M.
        template <class Tuple, size_t Index = std::tuple_size<Tuple>::value - 1>
        struct HashValueImpl
        {
            static void apply(size_t& seed, Tuple const& tuple)
            {
                HashValueImpl<Tuple, Index - 1>::apply(seed, tuple);
                hash_combine(seed, get<Index>(tuple));
            }
        };

        template <class Tuple>
        struct HashValueImpl<Tuple, 0>
        {
            static void apply(size_t& seed, Tuple const& tuple)
            {
                hash_combine(seed, get<0>(tuple));
            }
        };
    }

    template <typename ... TT>
    struct hash<std::tuple<TT...>>
    {
        size_t
            operator()(std::tuple<TT...> const& tt) const
        {
            size_t seed = 0;
            HashValueImpl<std::tuple<TT...> >::apply(seed, tt);
            return seed;
        }

    };
}
void convertSceneToMesh(
    const R3SceneNode* node, R3Mesh* mesh,
    std::unordered_map<std::tuple<float, float, float>, R3MeshVertex*>& position2Vertex,
    std::unordered_map<std::tuple<size_t, size_t, size_t>, R3MeshFace*>& index2Face,
    int& duplicateVertices, int& duplicateFaces)
{
    //recurse into children
    for (int i = 0; i < node->NChildren(); ++i) {
        convertSceneToMesh(node->Child(i), mesh, position2Vertex, index2Face, duplicateVertices, duplicateFaces);
    }

	//loop over elements
	for (int i=0; i<node->NElements(); ++i)
	{
		//loop over shapes
		for (int j=0; j<node->Element(i)->NShapes(); ++j)
		{
            const R3Shape* shape = node->Element(i)->Shape(j);
			//check if this shape is a triangle array
            if (shape->ClassID() != R3TriangleArray::CLASS_ID())
                continue;
            const R3TriangleArray* array = dynamic_cast<const R3TriangleArray*>(shape);
			for (int k=0; k<array->NVertices(); ++k)
			{
                const R3TriangleVertex* oldV = array->Vertex(k);
                auto posKey = std::make_tuple(oldV->position.X(), oldV->position.Y(), oldV->position.Z());
                if (position2Vertex.count(posKey) == 0) { //check for duplicate vertices
                    R3MeshVertex* newV = mesh->CreateVertex(oldV->Position(), oldV->Normal(), oldV->Color(), oldV->TextureCoords());
                    position2Vertex.emplace(posKey, newV);
                }
                else
                {
                    duplicateVertices++;
                }
			}
			for (int k=0; k<array->NTriangles(); ++k)
			{
                const R3Triangle* t = array->Triangle(k);
                R3MeshVertex* v[3];
                bool found = true;
				for (int l=0; l<3; ++l)
				{
                    auto posKey = std::make_tuple(t->Vertex(l)->position.X(), t->Vertex(l)->position.Y(), t->Vertex(l)->position.Z());
                    const auto it = position2Vertex.find(posKey);
                    if (it == position2Vertex.end()) { found = false; break; }
                    v[l] = it->second;
				}
                if (found) {
                    auto indexKey = sort3(
                        reinterpret_cast<size_t>(v[0]),
                        reinterpret_cast<size_t>(v[1]),
                        reinterpret_cast<size_t>(v[2]));
                    if (index2Face.count(indexKey) == 0) {
                        auto f = mesh->CreateFace(v[0], v[1], v[2]);
                        if (f != nullptr) {
                            f->category = 0;
                            index2Face.emplace(indexKey, f);
                        } else
                            duplicateFaces++;
                    }
                    else
                    {
                        duplicateFaces++;
                    }
                }
                else
                    printf("Unable to create triangle, no vertex found\n");
			}
		}
	}
}
float sqr(float x) {return x * x; };
float distSqr(const R3Point& a, const R3Point& b)
{
    return sqr(a.X() - b.X()) + sqr(a.Y() - b.Y()) + sqr(a.Z() - b.Z());
};
float triDistanceSqr(const R3MeshFace* in, const R3MeshFace* out)
{
    float dist = 0;
	for (int i=0; i<3; ++i)
	{
        float d = FLT_MAX;
        for (int j = 0; j < 3; ++j)
            d = min(d, distSqr(in->vertex[i]->position, out->vertex[j]->position));
        dist = max(dist, d);
	}
    return dist;
}
float triSizeSqr(const R3MeshFace* a)
{
    return distSqr(a->vertex[0]->position, a->vertex[1]->position)
		+ distSqr(a->vertex[2]->position, a->vertex[1]->position)
		+ distSqr(a->vertex[0]->position, a->vertex[2]->position);
}
void matchMeshes()
{
    printf("Match textured mesh 'scene' and segmented mesh 'mesh'\n");
	if (!house->mesh && !house->scene)
	{
        printf("No mesh or no scene loaded, can't continue\n");
        return;
	}
	if (!match_meshes || strlen(match_meshes)==0)
	{
        printf("No output file specified\n");
        return;
	}

	//1. Convert house->scene from R3TriangleArray into R3Mesh
    R3Mesh mesh;
    std::unordered_map<std::tuple<float, float, float>, R3MeshVertex*> position2Vertex;
    std::unordered_map<std::tuple<size_t, size_t, size_t>, R3MeshFace*> index2Face;
    int duplicateVertices = 0;
    int duplicateFaces = 0;
	for (int nodeID = 0; nodeID < house->scene->NNodes(); ++nodeID)
	{
        R3SceneNode* node = house->scene->Node(nodeID);
        convertSceneToMesh(node, &mesh, position2Vertex, index2Face, duplicateVertices, duplicateFaces);
	}
    printf("Output #vertices: %d, #faces: %d\n", mesh.NVertices(), mesh.NFaces());
    printf("duplicate vertices removed: %d\n", duplicateVertices);
    printf("duplicate faces removed: %d\n", duplicateFaces);
	
	//2. Copy category ids from the R3Mesh of house->mesh into the new R3Mesh
#if 1
    printf("Match and copy categories\n");
    int numTrisTarget = mesh.NFaces();
    int numTrisSource = house->mesh->NFaces();
    static const int batchSize = 32;
    for (int i1 = 0; i1 < numTrisTarget; i1 += batchSize)
    {
        printf("\r%7d / %7d", i1, numTrisTarget);
        fflush(stdout);
        const int end = min(numTrisTarget, i1 + batchSize);
#pragma omp parallel for
        for (int i = i1; i < end; ++i)
        {
            R3MeshFace* triOut = mesh.Face(i);
            //search for the triangle with the smallest distance
            float distance = FLT_MAX;
            int bestIndex = -1;
        	for (int j=0; j<numTrisSource; ++j)
        	{
                const R3MeshFace* triIn = house->mesh->Face(j);
                float dist = triDistanceSqr(triOut, triIn);
        		if (dist < distance)
        		{
                    bestIndex = j;
                    distance = dist;
        		}
        	}
            const static float EPSILON = 5;
        	if (distance <= EPSILON * triSizeSqr(triOut))
        	{
                triOut->category = house->mesh->Face(bestIndex)->category;
                auto color = GetColor(triOut->category);
                for (int l = 0; l < 3; ++l) {
                    triOut->vertex[l]->color = color;
                }
        	}
        }
    }
    printf("\n");
#endif
	
	//3. Save new mesh as .ply
    if (mesh.WritePlyFile(match_meshes, false) == 0)
        printf("Failed to write mesh\n");
    printf("Done\n");
}

////////////////////////////////////////////////////////////////////////
// Main function
////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  // Parse program arguments
  if (!ParseArgs(argc, argv)) exit(-1);

  // Allocate house
  house = new MPHouse();
  if (!house) {
    fprintf(stderr, "Unable to allocate house\n");
    exit(-1);
  }
  
  // Read house
  if (input_house_filename) {
    if (!ReadHouse(input_house_filename)) exit(-1);
  }

  // Read scene
  if (input_scene_filename) { //textured mesh
    if (!ReadScene(input_scene_filename)) exit(-1);
  }

  // Read mesh
  if (input_mesh_filename) {  //segmented mesh
    if (!ReadMesh(input_mesh_filename)) exit(-1);
  }

  // Read categories
  if (input_categories_filename) {
    if (!ReadCategories(input_categories_filename)) exit(-1);
  }

  // Read segments
  if (input_segments_filename) {
    if (!ReadSegments(input_segments_filename)) exit(-1);
  }

  // Read objects
  if (input_objects_filename) {
    if (!ReadObjects(input_objects_filename)) exit(-1);
  }

  // Read configuration
  if (input_configuration_filename) {
    if (!ReadConfiguration(input_configuration_filename)) exit(-1);
  }

  // Write house
  if (output_house_filename) {
    if (!WriteHouse(output_house_filename)) exit(-1);
  }

  if (match_meshes)
  {
      matchMeshes();
      return 0;
  }
  if (output_images_path)
  {
    GLUTInit(&argc, argv);
    batchWriteImages();
    return 0;
  }
	
  // Check if should start interactive viewer
  if (!batch || output_image_filename) {
    // Initialize GLUT
    GLUTInit(&argc, argv);

    // Run GLUT interface (does not return)
    GLUTMainLoop();
  }

  // Return success 
  return 0;
}



