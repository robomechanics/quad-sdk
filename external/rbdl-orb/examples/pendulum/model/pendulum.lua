--This is a 1 dof planar pendulum

print("Lua: Constructing 1 dof pendulum")

inertiaMatrix = {
    {0.0833,0.     ,0.    },
    {0.    ,0.00833,0.    },
    {0.    ,0.     ,0.0833}        
}

print("Lua: Body mass and geometry properties")
linkProperties= {   mass    = 1., 
                    com     = {0.,-1.,0.}, 
                    inertia = inertiaMatrix}

print("Lua: Putting bodies in a table")
bodies = {
    link  = linkProperties}

print("Lua: Making a table of joints")
--Is the point necessary?
joints = {
    rotational_z = {
        {0.,0.,1.,0.,0.,0.}   
    }
}

print("Lua: Making the meshes")
meshes = {
    link = {
        dimensions = { 0.05, 1.0, 0.05 },
        color = { 1, 0, 0 },
        mesh_center = { 0, -0.5, 0 },
        src = "unit_cube.obj",
    }
}

print("Lua: Making the model")
model = {
    gravity = {0., -9.81, 0.},

    frames = {    
        {
            name    = "K1",
            parent  = "ROOT",
            visuals = {meshes.link},
            body    = bodies.link,
            joint   = joints.rotational_z,
            joint_frame = {
                r = {0.5, 0, 0},
            }
        },
        {
            name    = "K2",
            parent  = "K1",
            joint_frame = {
                r = {0,-1, 0},
            }
        },

    }

}

return model
