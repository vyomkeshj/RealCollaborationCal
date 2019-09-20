//
// Created by controller on 2/12/19.
//

#ifndef PROJECT_PHOXICONVERSIONS_H
#define PROJECT_PHOXICONVERSIONS_H

#include <PhoXi.h>
#include <headers/PhoXiDeviceInformation.h>

void toPhoXiCameraDeviceInforamtion(const pho::api::PhoXiDeviceInformation& phoXiDeviceInformation,
                                    phoxi_camera::PhoXiDeviceInformation& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.name = phoXiDeviceInformation.Name;
    phoXiCameraDeviceInformation.type = phoxi_camera::PhoXiDeviceInformation::PhoXiDeviceType(
            (int)phoXiDeviceInformation.Type);
    phoXiCameraDeviceInformation.hwIdentification = phoXiDeviceInformation.HWIdentification;
    if (phoXiDeviceInformation.Status.Ready == true) {
        phoXiCameraDeviceInformation.status = phoXiDeviceInformation.Status.Ready
                                              ? phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Ready
                                              : phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Starting;
    } else {
        phoXiCameraDeviceInformation.status = phoXiDeviceInformation.Status.Attached
                                              ? phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Ready
                                              : phoxi_camera::PhoXiDeviceInformation::PhoXiConnectionStatus::Occupied;
    }
    phoXiCameraDeviceInformation.firmwareVersion = phoXiDeviceInformation.FirmwareVersion;
}

void toPhoXiCameraDeviceInforamtion(const std::vector<pho::api::PhoXiDeviceInformation>& phoXiDeviceInformation,
                                    std::vector<phoxi_camera::PhoXiDeviceInformation>& phoXiCameraDeviceInformation) {
    phoXiCameraDeviceInformation.clear();
    phoXiCameraDeviceInformation.resize(phoXiDeviceInformation.size());
    for (int i = 0; i < phoXiDeviceInformation.size(); ++i) {
        toPhoXiCameraDeviceInforamtion(phoXiDeviceInformation[i], phoXiCameraDeviceInformation[i]);
    }
}

#endif //PROJECT_PHOXICONVERSIONS_H
