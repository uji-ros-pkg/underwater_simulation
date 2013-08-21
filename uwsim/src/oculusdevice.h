/*
 * oculusdevice.h
 *
 *  Created on: Jul 03, 2013
 *      Author: Bjorn Blissing
 */
 
#ifndef _OSG_OCULUSDEVICE_H_
#define _OSG_OCULUSDEVICE_H_

#include <OVR.h>
#include <osg/Matrix>


class OculusDevice {

	public:
		OculusDevice();
		~OculusDevice();

		enum EyeSide {
			CENTER_EYE	= 0,
			LEFT_EYE	= 1,
			RIGHT_EYE	= 2
		};

		unsigned int hScreenResolution() const;
		unsigned int vScreenResolution() const;
		float hScreenSize() const;
		float vScreenSize() const;
		float vScreenCenter() const;
		float eyeToScreenDistance() const;
		float lensSeparationDistance() const;
		float interpupillaryDistance() const;

		osg::Matrix viewMatrix(EyeSide eye = CENTER_EYE) const;
		osg::Matrix projectionMatrix(EyeSide eye = CENTER_EYE) const;
        osg::Matrix projectionOffsetMatrix(EyeSide eye = CENTER_EYE) const;
        osg::Matrix projectionCenterMatrix() const;

		osg::Vec2f lensCenter(EyeSide eye = CENTER_EYE) const;
		osg::Vec2f screenCenter() const;
		osg::Vec4f warpParameters() const;
		osg::Vec4f chromAbParameters() const;
		osg::Vec2f scale() const;
		osg::Vec2f scaleIn() const;

		void setScaleFactor(float scaleFactor) { m_scaleFactor = scaleFactor; }
		float scaleFactor() const { return m_scaleFactor; }

		float aspectRatio() const {  return float (hScreenResolution()/2) / float (vScreenResolution()); }
		osg::Quat getOrientation() const;

		void setNearClip(float nearClip) { m_nearClip = nearClip; }
		void setFarClip(float farclip) { m_farClip = farclip; }

	protected:
		float viewCenter() const { return hScreenSize() * 0.25f; }
		float halfIPD() const { return interpupillaryDistance() * 0.5f; }

		OVR::DeviceManager* m_deviceManager;
		OVR::HMDDevice* m_hmdDevice;
		OVR::HMDInfo* m_hmdInfo;
		OVR::SensorFusion m_sensorFusion;
		float m_scaleFactor;
		float m_nearClip;
		float m_farClip;
	private:
		OculusDevice(const OculusDevice&); // Do not allow copy
};

#endif
