#include "train_tracking/traffic_manager.h"
#include "traffic_manager_internal.h"

void traffic_init(void) {
    traffic_reservation_init();
    traffic_attr_init();
}
