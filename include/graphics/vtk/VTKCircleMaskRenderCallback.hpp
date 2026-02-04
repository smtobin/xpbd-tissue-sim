#ifndef __VTK_CIRCLE_MASK_RENDER_CALLBACK_HPP
#define __VTK_CIRCLE_MASK_RENDER_CALLBACK_HPP

#include "common/types.hpp"

#include <vtkCommand.h>
#include <vtkCamera.h>

#include "graphics/vtk/VTKViewer.hpp"

namespace Graphics
{

/** A custom callback that will resize the circle mask used by the VTKViewer when the size of the window changes.
 * It is called every time the window renders (to avoid a race condition on when called on a window resize).
 * We store the last width and height to avoid recreating the mask every frame.
 */
class VTKCircleMaskRenderCallback : public vtkCommand
{
public:
    static VTKCircleMaskRenderCallback* New() { return new VTKCircleMaskRenderCallback; }

    VTKViewer* viewer = nullptr;

    int last_w = -1;
    int last_h = -1;

    void Execute(vtkObject* caller, unsigned long, void*) override
    {
        auto* rw = vtkRenderWindow::SafeDownCast(caller);
        if (!rw || !viewer)
            return;

        int* size = rw->GetSize();

        if (size[0] != last_w || size[1] != last_h)
        {
            last_w = size[0];
            last_h = size[1];
            viewer->updateCircleMask();
        }
    }
};

} // namespace Graphics

#endif // __VTK_CIRCLE_MASK_RENDER_CALLBACK_HPP