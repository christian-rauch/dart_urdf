
#include <read_model_urdf.h>

// urdf parser and model
#include <urdf_parser/urdf_parser.h>

#ifdef ROS_RESOURCE
    #include <ros/package.h>
#else
    #include <boost/algorithm/string/replace.hpp>
#endif
#include <boost/filesystem.hpp>

// constants for determining and replacing the package path in URDF mesh paths
#define PACKAGE_PATH_FILE "package.xml"
#define PACKAGE_PATH_URI_SCHEME "package://"

//#define PRNT_DBG

namespace dart {

/// type definitions for shared pointers
typedef const std::shared_ptr< const urdf::ModelInterface> ModelInterfaceConstPtr;

typedef std::shared_ptr< urdf::Link > LinkPtr;
typedef std::shared_ptr< const urdf::Link > LinkConstPtr;
typedef std::shared_ptr< urdf::Joint > JointPtr;
typedef std::shared_ptr< const urdf::Joint > JointConstPtr;

typedef std::vector< std::shared_ptr< urdf::Link > > LinkPtrVec;
typedef std::vector< std::shared_ptr< urdf::Joint > > JointPtrVec;

/**
 * @brief The MeshLoaderConfig struct
 */
struct MeshLoaderConfig {

    /**
     * @brief package_path store absolute path to URDF package rot directory
     */
    std::string package_path;

    /**
     * @brief colour RGB colour definition for untextured meshes
     */
    std::vector<uint8_t> colour;
};

/**
 * @brief extract_joints get list of joints from std::map
 * @param joint_map dictionary containing joint name and pointer
 * @return vector of joint pointers
 */
JointPtrVec extract_joints(const std::map< std::string, JointPtr > &joint_map) {
    JointPtrVec joints;
    for(auto jm: joint_map) { joints.push_back(jm.second); }
    return joints;
}

/**
 * @brief print_joints print the name of all joints in given list
 * @param joints vector of joints
 */
void print_joints(const JointPtrVec &joints) {
    std::cout<<"joints total: "<<joints.size()<<std::endl;
    for(auto j: joints) { std::cout<<"joint: "<<j->name<<std::endl; }
}

/**
 * @brief print_links print the name of all links in given list
 * @param links vector of links
 */
void print_links(const LinkPtrVec &links) {
    std::cout<<"links total: "<<links.size()<<std::endl;
    for(auto l: links) { std::cout<<"link: "<<l->name<<std::endl; }
}

/**
 * @brief extract_frames process link and child joints recursively
 * @param parent_id ID of parrent frame, that called this function
 * @param link parent link
 * @param urdf_model URDF model to read additional information from
 * @param model DART model that is extended with links and joints of URDF model
 * @param conf configuration structure to store permanent and global information
 * @return true on success
 * @return false on failure
 */
bool extract_frames(const int parent_id, LinkConstPtr &link, ModelInterfaceConstPtr &urdf_model, HostOnlyModel &model, const MeshLoaderConfig &conf) {

    JointPtrVec joints = link->child_joints;

#ifdef PRNT_DBG
    std::cout<<"id: "<<parent_id<<" (link: "<<link->name<<", joints: "<<joints.size()<<")"<<std::endl;
#endif

    /////////////////////////////////////////////////////
    /// get geometric volume

    if(link->visual != NULL) {
        std::shared_ptr< urdf::Geometry > geometry = link->visual->geometry;

        dart::GeomType geom_type;
        std::string sx, sy, sz; // scale
        std::string tx, ty, tz; // translation
        std::string rx, ry, rz; // rotation
        uint8_t r, g, b;        // colour
        std::string mesh_path = "";

        // translation
        urdf::Vector3 pos = link->visual->origin.position;
        tx = std::to_string(pos.x);
        ty = std::to_string(pos.y);
        tz = std::to_string(pos.z);

        // rotation
        urdf::Rotation rot = link->visual->origin.rotation;
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);
        // right-hand coordinate system
        rx = std::to_string(roll);    // roll: rotation around x-axis (facing forward)
        ry = std::to_string(pitch);   // pitch: rotation around y-axis (facing right)
        rz = std::to_string(yaw);     // yaw: rotation around z-axis (facing down)

        switch(geometry->type) {
        case urdf::Geometry::SPHERE: {
            geom_type = PrimitiveSphereType;
            double radius = dynamic_cast<urdf::Sphere*>(&*geometry)->radius;
            sx = std::to_string(radius);
            sy = std::to_string(radius);
            sz = std::to_string(radius);
        }
            break;
        case urdf::Geometry::BOX: {
            geom_type = PrimitiveCubeType;
            urdf::Vector3 size = dynamic_cast<urdf::Box*>(&*geometry)->dim;
            sx = std::to_string(size.x);
            sy = std::to_string(size.y);
            sz = std::to_string(size.z);
        }
            break;
        case urdf::Geometry::CYLINDER: {
            geom_type = PrimitiveCylinderType;
            double radius = dynamic_cast<urdf::Cylinder*>(&*geometry)->radius;
            double length = dynamic_cast<urdf::Cylinder*>(&*geometry)->length;
            sx = std::to_string(radius);
            sy = std::to_string(radius);
            sz = std::to_string(length);

            // workaround: transform cylinder origin from URDF specification
            // (origin at centre of cylinder volume) to DART (centre of bottom)
            tz = std::to_string(pos.z - (length/2));
        }
            break;
        case urdf::Geometry::MESH: {
            geom_type = MeshType;
            // geometry is a pointer to base class, we need to cast (*geometry)
            mesh_path = dynamic_cast<urdf::Mesh*>(&*geometry)->filename;
            urdf::Vector3 scale = dynamic_cast<urdf::Mesh*>(&*geometry)->scale;
            sx = std::to_string(scale.x);
            sy = std::to_string(scale.y);
            sz = std::to_string(scale.z);

            if(mesh_path.find(PACKAGE_PATH_URI_SCHEME) != std::string::npos) {
#ifdef ROS_RESOURCE
                mesh_path.erase(0, strlen(PACKAGE_PATH_URI_SCHEME));
                const size_t pos = mesh_path.find("/");
                const std::string package_name = mesh_path.substr(0, pos);
                mesh_path.erase(0, pos);
                const std::string package_path = ros::package::getPath(package_name);

                if (package_path.empty()) {
                    throw std::runtime_error("Package [" + package_name + "] does not exist");
                }

                mesh_path = package_path + mesh_path;
#else
                // we need to replace "package://" by full path
                boost::algorithm::replace_first(mesh_path, PACKAGE_PATH_URI_SCHEME, conf.package_path);
#endif
            }
            else {
                // prepend full path
                mesh_path = conf.package_path + mesh_path;
            }

#ifdef PRNT_DBG
            std::cout<<"loading mesh from: "<<mesh_path<<std::endl;
#endif
        }
            break;
        default:
            std::cerr<<"unknown geometry"<<std::endl;
        }

        // colour
        if(conf.colour.size()!=0) {
            r = conf.colour[0];
            g = conf.colour[1];
            b = conf.colour[2];
        }
        else if(link->visual->material!=NULL){
            urdf::Color colour = link->visual->material->color;
            r = colour.r*255;
            g = colour.g*255;
            b = colour.b*255;
        }
        else {
            r = 127;
            g = 127;
            b = 127;
        }

        model.addGeometry(parent_id, geom_type,
                          sx, sy, sz,
                          tx, ty, tz,
                          rx, ry, rz,
                          r, g, b,
                          mesh_path);

    }

    /////////////////////////////////////////////////////
    /// get joint attributes
    for(JointPtr j: joints) {
        // joint attributes expected by DART
        std::string name;                   // joint name
        dart::JointType type;               // type
        std::string min, max;               // joint limits
        std::string posX, posY, posZ;       // 3D position
        std::string oriX, oriY, oriZ;       // 3D orientation
        std::string axisX, axisY, axisZ;    // rotation axis

        // name
        name = j->name;

#ifdef PRNT_DBG
        std::cout<<"processing joint: "<<name<<std::endl;
#endif

        // joint type
        switch(j->type) {
        case urdf::Joint::REVOLUTE:
        case urdf::Joint::CONTINUOUS:
        case urdf::Joint::FIXED:
            // DART does not support unknown or fixed joints
            // we assume these are revolute joints
            type = dart::RotationalJoint;
            break;
        case urdf::Joint::PRISMATIC:
            type = dart::PrismaticJoint;
            break;
        case urdf::Joint::UNKNOWN:
        case urdf::Joint::FLOATING:
        case urdf::Joint::PLANAR:
            std::cerr<<"ignoring joint '" << j->name << "' with unsupported type ("<<j->type<<")"<<std::endl;
            break;
        }

        // position
        urdf::Vector3 pos = j->parent_to_joint_origin_transform.position;
        posX = std::to_string(pos.x);
        posY = std::to_string(pos.y);
        posZ = std::to_string(pos.z);

        // orientation
        urdf::Rotation rot = j->parent_to_joint_origin_transform.rotation;
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);
        // right-hand coordinate system
        oriX = std::to_string(roll);    // roll: rotation around x-axis (facing forward)
        oriY = std::to_string(pitch);   // pitch: rotation around y-axis (facing right)
        oriZ = std::to_string(yaw);     // yaw: rotation around z-axis (facing down)

        // rotation axis
        urdf::Vector3 axis = j->axis;
        axisX = std::to_string(axis.x);
        axisY = std::to_string(axis.y);
        axisZ = std::to_string(axis.z);

        // joint limits
        switch(j->type) {
        case urdf::Joint::REVOLUTE:
        case urdf::Joint::PRISMATIC:
            // joint limits are required
            min = std::to_string(j->limits->lower);
            max = std::to_string(j->limits->upper);
            break;
        case urdf::Joint::UNKNOWN:
        case urdf::Joint::FIXED:
            // joint limits are optional
            if(j->limits!=nullptr) {
                min = std::to_string(j->limits->lower);
                max = std::to_string(j->limits->upper);
            }
            else {
                min = "0";
                max = "0";
            }
            break;
        case urdf::Joint::CONTINUOUS:
            // continous movement without limits
            // we can not set infinite bounds, use mutiples of full rotations
            min = std::to_string(0);
            max = std::to_string(4*M_PI);
            break;
        case urdf::Joint::FLOATING:
        case urdf::Joint::PLANAR:
            min = "0";
            max = "0";
            std::cerr<<"ignoring limits of joint '" << j->name << "' (type: "<<j->type<<")"<<std::endl;
            break;
        }

        // add frame with attributes
        const int child_id =
            model.addFrame(parent_id, type, posX, posY, posZ, oriX, oriY, oriZ,
                           axisX, axisY, axisZ, min, max, name);
        model.addFrameName(j->child_link_name);

        // follow edges
        LinkConstPtr l_child = urdf_model->getLink(j->child_link_name);
        extract_frames(child_id, l_child, urdf_model, model, conf);
    } // iteration of joints

    // nothing exceptional happened so far
    return true;
}

/**
 * @brief readModelURDF readModelURDF parse URDF model description and store kinematic and meshes in DART model format
 */
bool readModelURDF(const ModelInterfaceConstPtr urdf_model,
                   const std::string &path,
                   HostOnlyModel &model,
                   const std::string &root_link_name,
                   const std::vector<uint8_t> &colour)
{
    // get robot name
    std::cout<<"found URDF robot: "<<urdf_model->getName()<<std::endl;
    model.setName(urdf_model->getName());

    // fix DART model version (currently 0 and above is checked)
    model.setModelVersion(1);

    // get root link
    std::shared_ptr<const urdf::Link> l_root;
    l_root = root_link_name.empty()? urdf_model->getRoot() : urdf_model->getLink(root_link_name);

    if(l_root!=NULL) {
        std::cout<<"root link: "<<l_root->name<<std::endl;

        MeshLoaderConfig conf;

        conf.colour = colour;

#ifndef ROS_RESOURCE
        // get full absolute path
        boost::filesystem::path fpath = boost::filesystem::canonical(path);

        // search backwards for package path, e.g. directory that contains the package file
        while(fpath.has_parent_path() && !boost::filesystem::is_regular_file(fpath / PACKAGE_PATH_FILE)) {
            // go one step backward closer to root
            fpath = fpath.parent_path();
        }

        if(!boost::filesystem::is_regular_file(fpath / PACKAGE_PATH_FILE)) {
            // package path not found, use relative path
            conf.package_path = boost::filesystem::canonical(path).parent_path().native()
                    + boost::filesystem::path::preferred_separator;
        }
        else {
            // store package path with trailing directory seperator
            conf.package_path = fpath.branch_path().native() + boost::filesystem::path::preferred_separator;
        }
        std::cout<<"URDF package path: "<<conf.package_path<<std::endl;
#endif

        // extract links and joints recursively
        model.addFrameName(l_root->name);
        return extract_frames(0, l_root, urdf_model, model, conf);
    }
    else {
        std::cerr<<"could not find root link "<<root_link_name<<std::endl;
        return false;
    }
}

bool readModelURDF(const std::string &path,
                   HostOnlyModel &model,
                   const std::string &root_link_name,
                   const std::vector<uint8_t> &colour)
{
    // parse URDF file
    const ModelInterfaceConstPtr urdf_model = urdf::parseURDFFile(path);
    return readModelURDF(urdf_model, path, model, root_link_name, colour);
}

bool readModelURDFxml(const std::string &xml_string,
                   HostOnlyModel &model,
                   const std::string &root_link_name,
                   const std::vector<uint8_t> &colour)
{
    // parse URDF string
    ModelInterfaceConstPtr urdf_model = urdf::parseURDF(xml_string);
    return readModelURDF(urdf_model, "", model, root_link_name, colour);
}

const HostOnlyModel &readModelURDF(const std::string &path,
                                   const std::string &root_link_name,
                                   const std::vector<uint8_t> &colour)
{
    HostOnlyModel *model = new HostOnlyModel();
    readModelURDF(path, *model, root_link_name, colour);
    return *model;
}

const HostOnlyModel &readModelURDFxml(const std::string &xml_string,
                                      const std::string &root_link_name,
                                      const std::vector<uint8_t> &colour)
{
    HostOnlyModel *model = new HostOnlyModel();
    readModelURDFxml(xml_string, *model, root_link_name, colour);
    return *model;
}

} // namespace dart
