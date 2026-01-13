#ifndef __GRAPHICS_OBJECT_HPP
#define __GRAPHICS_OBJECT_HPP

#include <string>
#include <iostream>

namespace Graphics
{

/** Abstract base class for all graphics objects
 * 
 * - Stored in a GraphicsScene for visualization.
 * - The update() method will update any graphics buffers (i.e. vertex buffers, etc.)
 *    so that the GraphicsScene has the most up-to-date information before rendering.
 */
class GraphicsObject 
{
    public:
    /** Creates a GraphicsObject with the given name
     * @param name : the name of the new GraphicsObject
     */
    explicit GraphicsObject(const std::string& name)
        : _name(name) {}

    virtual ~GraphicsObject() {};
    
    /** Updates the latest "snapshot" of the simulation state that should be rendered. */
    virtual void update() = 0;

    /** Updates graphics buffers associated with this object */
    virtual void updateGraphicsBuffers() = 0;

    /** Returns the name of this GraphicsObject
     * @returns the name of this GraphicsObject
     */
    std::string name() const { return _name; }

    protected:
    /** Name of this GraphicsObject */
    std::string _name;
};

} // namespace Graphics

#endif // __GRAPHICS_OBJECT_HPP