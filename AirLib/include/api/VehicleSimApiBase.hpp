// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleSimApiBase_hpp
#define air_VehicleSimApiBase_hpp

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/ImageCaptureBase.hpp"
#include "physics/Kinematics.hpp"
#include "physics/Environment.hpp"
#include "common/AirSimSettings.hpp"

namespace msr { namespace airlib {

class VehicleSimApiBase : public msr::airlib::UpdatableObject {
public:
    virtual ~VehicleSimApiBase() = default;

    //default implementation so derived class doesn't have to call on UpdatableObject
    virtual void reset() override
    {
        UpdatableObject::reset();
    }
    virtual void update() override
    {
        UpdatableObject::update();
    }

    //this method is called at every render tick when we want to transfer state from
    //physics engine to render engine. As physics engine is halted while 
    //this happens, this method should do minimal processing
    virtual void updateRenderedState(float dt)
    {
        unused(dt);
        //derived class should override if needed
    }
    //called when render changes are required at every render tick
    virtual void updateRendering(float dt)
    {
        unused(dt);
        //derived class should override if needed
    }

    virtual const ImageCaptureBase* getImageCapture() const = 0;
    virtual ImageCaptureBase* getImageCapture()
    {
        return const_cast<ImageCaptureBase*>(static_cast<const VehicleSimApiBase*>(this)->getImageCapture());
    }

    virtual void initialize() = 0;

    virtual std::vector<ImageCaptureBase::ImageResponse> getImages(const std::vector<ImageCaptureBase::ImageRequest>& request) const = 0;
    virtual std::vector<uint8_t> getImage(const std::string& camera_name, ImageCaptureBase::ImageType image_type) const = 0;

    virtual Pose getPose() const = 0;
    virtual void setPose(const Pose& pose, bool ignore_collision) = 0;

    virtual Boundary getBoundary() const = 0;
    virtual void setBoundary(const Boundary& boundary) = 0;
    virtual void enableCustomBoundaryData(bool is_enable) = 0;

    virtual const Kinematics::State* getGroundTruthKinematics() const = 0;
    virtual const msr::airlib::Environment* getGroundTruthEnvironment() const = 0;

    virtual CameraInfo getCameraInfo(const std::string& camera_name) const = 0;
    virtual void setCameraOrientation(const std::string& camera_name, const Quaternionr& orientation) = 0;

    virtual CollisionInfo getCollisionInfo() const = 0;
    virtual int getRemoteControlID() const = 0; //which RC to use, 0 is first one, -1 means disable RC (use keyborad)
    virtual RCData getRCData() const = 0; //get reading from RC from simulator's host OS
    virtual std::string getVehicleName() const = 0;
    virtual std::string getRecordFileLine(bool is_header_line) const = 0;
    virtual void toggleTrace() = 0;
    virtual void applyDisturbance(bool left) = 0;
    virtual void toggleBoundary() = 0;

    //use pointer here because of derived classes for VehicleSetting
    const AirSimSettings::VehicleSetting* getVehicleSetting() const
    {
        return AirSimSettings::singleton().getVehicleSetting(getVehicleName());
    }
};

} } //namespace
#endif
