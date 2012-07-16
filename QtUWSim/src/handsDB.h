#include <string>
#include <vector>

#include <database_interface/db_class.h>

class Hands : public database_interface::DBClass{
public:
		database_interface::DBField<int> hand_id;
		database_interface::DBField<std::string> name_hand;
		database_interface::DBField<std::string> package_ros;
		database_interface::DBField<std::string> path;
		
		Hands() :
			hand_id(database_interface::DBFieldBase::TEXT, this, "hand_id", "hands", true),
			name_hand(database_interface::DBFieldBase::TEXT, this, "name_hand", "hands", true),
			package_ros(database_interface::DBFieldBase::TEXT, this, "package_ros", "hands", true),
			path(database_interface::DBFieldBase::TEXT, this, "path", "hands", true)
		{
			primary_key_field_=&hand_id;
			fields_.push_back(&name_hand);
			fields_.push_back(&package_ros);
			fields_.push_back(&path);
			
			setAllFieldsReadFromDatabase(true);
			setAllFieldsWriteToDatabase(true);
		}
};



class Shapes:public database_interface::DBClass{
public:
		database_interface::DBField<int> shape_id;
		database_interface::DBField<std::string> name_shape;
		database_interface::DBField<int> hand_id;
		
		Shapes() :
			shape_id(database_interface::DBFieldBase::TEXT, this, "shape_id", "shapes", true),
			name_shape(database_interface::DBFieldBase::TEXT, this, "name_shapes", "shapes", true),
			hand_id(database_interface::DBFieldBase::TEXT, this, "hand_id", "shapes", true)
		{
			primary_key_field_=&shape_id;
			fields_.push_back(&name_shape);
			fields_.push_back(&hand_id);
			
			setAllFieldsReadFromDatabase(true);
			setAllFieldsWriteToDatabase(true);
		}
};

class Joints:public database_interface::DBClass{
public:
		database_interface::DBField<int> joint_id;
		database_interface::DBField<std::string> name_joint;
		database_interface::DBField<double> position;
		database_interface::DBField<int> shape_id;
		
		Joints() :
			joint_id(database_interface::DBFieldBase::TEXT, this, "joint_id", "joints", true),
			name_joint(database_interface::DBFieldBase::TEXT, this, "name_joint", "joints", true),
			position(database_interface::DBFieldBase::TEXT, this, "position", "joints", true),
			shape_id(database_interface::DBFieldBase::TEXT, this, "shape_id", "joints", true)
		{
			primary_key_field_=&joint_id;
			fields_.push_back(&name_joint);
			fields_.push_back(&position);
			fields_.push_back(&shape_id);
			
			setAllFieldsReadFromDatabase(true);
			setAllFieldsWriteToDatabase(true);
		}
};