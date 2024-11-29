import maya.cmds as cmds

def import_files(train_path, carriage_path, num_carriages, add_train_at_end):
    try:
        train_group, carriage_group = None, None
        
        # Import and group the train
        if train_path and cmds.file(train_path, query=True, exists=True):
            train_nodes = cmds.file(train_path, i=True, returnNewNodes=True)
            train_group = cmds.group(train_nodes, name="Train_Group")
        else:
            cmds.warning("Train file path is invalid or does not exist.")
        
        # Import and group the carriage
        if carriage_path and cmds.file(carriage_path, query=True, exists=True):
            carriage_nodes = cmds.file(carriage_path, i=True, returnNewNodes=True)
            carriage_group = cmds.group(carriage_nodes, name="Carriage_Group")
        else:
            cmds.warning("Carriage file path is invalid or does not exist.")
        
        # Align pivots, freeze transformations, and reposition
        if train_group and carriage_group:
            align_pivots(train_group, carriage_group)
            freeze_transformations(train_group, carriage_group)
            groups = position_carriages(train_group, carriage_group, num_carriages, add_train_at_end)
            all_locators = []
            for group in groups:
                all_locators.extend(add_locators_to_wheels(group))
            print("Locators:", all_locators)

            first_joint, last_joint = create_skeleton_from_locators(all_locators)

            # Look for an object called "Tracks"
            tracks = cmds.ls("Tracks", long=True)

            # Ensure tracks is a valid curve
            if tracks:
                # Create the IK Spline Handle
                ik_spline_handle = cmds.ikHandle(
                    startJoint=last_joint,
                    endEffector=first_joint,
                    curve=tracks[0],  # Use the first curve found
                    solver="ikSplineSolver",
                    createCurve=False,  # Don't create a new curve, just use the existing one
                    simplifyCurve=True  # Optionally simplify the curve for better performance
                )
                cmds.select(ik_spline_handle[0])  # Select the IK handle for further operations if needed
                print(f"IK Spline Handle created with joints {first_joint} to {last_joint} and track {tracks[0]}")
            else:
                cmds.warning("No tracks curve found. Please create a curve named 'Tracks' in the scene.")

    except Exception as e:
        cmds.error(f"Error importing files: {e}")


def create_skeleton_from_locators(all_locators):
    if not all_locators:
        cmds.warning("No locators found, skeleton creation skipped.")
        return
    
    # Initialize an empty list to store joints
    joints = []

    final_parent = []
    
    # Iterate through the locators and create joints
    for i in range(len(all_locators)):
        locator_position = cmds.xform(all_locators[i], query=True, ws=True, t=True)
        
        # Get pos1 which is 1 unit positive z of the locator
        pos1 = [locator_position[0], locator_position[1], locator_position[2] + 1]
        
        # Get pos2 which is 1 unit negative z of the locator
        pos2 = [locator_position[0], locator_position[1], locator_position[2] - 1]
        
        # Create a joint at pos1 and a joint at pos2
        cmds.select(clear=True)  # Clear selection to create new joint
        joint1 = cmds.joint(p=pos1)  # Create joint at pos1
        joint2 = cmds.joint(p=pos2)  # Create joint at pos2
        
        # Append the new joints to the list
        joints.append(joint1)
        joints.append(joint2)

        final_parent.append([all_locators[i],joint2])

    print("Joints:", joints)
    
    # Unparent all joints before reparenting them
    for joint in joints:
        cmds.parent(joint, world=True)  # Unparent each joint from its previous parent

    print("Joints unparented.")
    
    # Sort the joints by their z position
    joints_sorted = sorted(joints, key=lambda joint: cmds.xform(joint, query=True, ws=True, t=True)[2])

    first_joint = joints_sorted[-1]
    last_joint = joints_sorted[0]

    print("Sorted:", joints_sorted)
    
    # Parent the joints in order based on their z position (first joint becomes the root)
    for i in range(1, len(joints_sorted)):
        cmds.parent(joints_sorted[i], joints_sorted[i-1])  # Parent each joint to the previous one
    
    print("Skeleton creation complete!")

    for selection in final_parent:
        cmds.parent(selection[0], selection[1])
        print("Parented:",selection[0],selection[1])

    return first_joint, last_joint


def add_locators_to_wheels(group):
    # Find all child objects in the group (excluding the group itself)
    wheel_objects = cmds.listRelatives(group, children=True, fullPath=True)
    added_locators = []  # Track added locators
    
    if wheel_objects:  # Check if there are any children
        for obj in wheel_objects:
            if "wheel" in obj.lower():
                # Get the pivot point of the object
                pivot = cmds.xform(obj, query=True, ws=True, rp=True)
                # Create a locator and place it at the pivot point
                locator = cmds.spaceLocator(name=f"{obj}_wheel_locator")[0]
                cmds.xform(locator, translation=pivot, ws=True)
                # Parent the locator to the group (not the wheel)
                cmds.parent(locator, group)
                cmds.parent(obj, locator)
                added_locators.append(locator)  # Keep track of the locators

        for obj in wheel_objects:
            if "body" in obj.lower():
                # Set the object's pivot point to one of the locators (assuming first locator)
                if added_locators:
                    # Set pivot to the first locator
                    cmds.xform(obj, piv=cmds.xform(added_locators[0], query=True, ws=True, rp=True), ws=True)
                    cmds.pointConstraint(added_locators[0], obj, maintainOffset=True)
                    cmds.aimConstraint(added_locators[1], obj, maintainOffset=True)

    else:
        cmds.warning(f"No children found in group {group}.")
    
    if not added_locators:
        cmds.warning("No wheels found in the group to add locators.")

    return added_locators

def align_pivots(train_group, carriage_group):
    target_position = [0, 0, 0]
    
    # Move train and carriage groups to origin based on pivot
    move_to_world_pivot(train_group, target_position)
    move_to_world_pivot(carriage_group, target_position)

def move_to_world_pivot(group, target_position):
    current_pivot = cmds.xform(group, query=True, ws=True, rp=True)
    offset = [target_position[i] - current_pivot[i] for i in range(3)]
    cmds.xform(group, t=offset, r=True)

def freeze_transformations(*groups):
    for group in groups:
        cmds.makeIdentity(group, apply=True, t=1, r=1, s=1, n=0)
        cmds.xform(group, piv=[0, 0, 0], ws=True)  # Reset pivot to origin after freezing

def position_carriages(train_group, carriage_group, num_carriages, add_train_at_end):
    # Get train's bounding box to determine its size
    train_bbox = cmds.xform(train_group, query=True, bb=True, ws=True)
    
    # Calculate the length of the train in Z (front-to-back direction)
    train_length = train_bbox[5] - train_bbox[2]
    
    # Move the first carriage to the back of the train
    cmds.move(-train_length, carriage_group, z=True, r=True)

    # Get carriage's bounding box to determine its size
    carriage_bbox = cmds.xform(carriage_group, query=True, bb=True, ws=True)

    # Calculate the length of the carriage in Z (front-to-back direction)
    carriage_length = carriage_bbox[5] - carriage_bbox[2]

    # Keeps track of the last added carriage position
    last_position = None

    # List to track train and all carriages (original and duplicates)
    all_groups = [train_group, carriage_group]

    # Duplicate the carriage and move it behind each other
    for i in range(1, num_carriages):
        new_carriage = cmds.duplicate(carriage_group, name=f"Carriage_Group{i + 1}")[0]
        # Position each carriage relative to the previous one
        cmds.move(-carriage_length * i, new_carriage, z=True, r=True)
        # Update last carriage
        last_position = -carriage_length * i
        all_groups.append(new_carriage)  # Add the new carriage to the list
    
    # If the checkbox is checked, move the train to the end of the last carriage
    if add_train_at_end:
        new_train = cmds.duplicate(train_group, name=f"Train_Group")[0]
        cmds.move(last_position-(train_length * 2), new_train, z=True, r=True)
        cmds.rotate(0, 180, 0, new_train, relative=True)
        all_groups.append(new_train)  # Add the new train to the list
    
    cmds.confirmDialog(title="Positioning Complete", 
                       message=f"{num_carriages} Carriages moved behind Train.", button=["OK"])

    return all_groups  # Return the list of all groups


def create_import_gui():
    window_name = "TrainCarriageImporter"
    if cmds.window(window_name, exists=True):
        cmds.deleteUI(window_name)
    
    window = cmds.window(window_name, title="Import Train and Carriage", widthHeight=(400, 250))
    cmds.columnLayout(adjustableColumn=True, columnAlign="center")

    # Train file field
    cmds.text(label="Train .mb File:", align="left")
    train_field = cmds.textFieldButtonGrp(
        buttonLabel='Browse...',
        placeholderText="Select Train File...",
        buttonCommand=lambda: select_file(train_field)
    )

    # Carriage file field
    cmds.text(label="Carriage .mb File:", align="left")
    carriage_field = cmds.textFieldButtonGrp(
        buttonLabel='Browse...',
        placeholderText="Select Carriage File...",
        buttonCommand=lambda: select_file(carriage_field)
    )

    # Number of carriages input
    cmds.text(label="Number of Carriages:", align="left")
    num_carriages_field = cmds.intField(minValue=1, value=1)

    # Checkbox for adding train at the end of the carriages
    cmds.text(label="Add Train at End of Carriages:", align="left")
    add_train_checkbox = cmds.checkBox(label="Yes", value=False)

    # Import button
    cmds.button(label="Import Files", command=lambda _: import_files(
        cmds.textFieldButtonGrp(train_field, query=True, text=True),
        cmds.textFieldButtonGrp(carriage_field, query=True, text=True),
        cmds.intField(num_carriages_field, query=True, value=True),
        cmds.checkBox(add_train_checkbox, query=True, value=True)
    ))

    cmds.showWindow(window)

def select_file(field):
    file_path = cmds.fileDialog2(fileMode=1, caption="Select File")
    if file_path:
        cmds.textFieldButtonGrp(field, edit=True, text=file_path[0])

create_import_gui()
