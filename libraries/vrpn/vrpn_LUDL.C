// Device drivers for the LUDL family of translation stage controllers.

// XXX If we don't recenter, then we definitely need a way to figure out where
//     our analogs are.
// XXX Need to parse more than one report if there is more than one in the buffer
//      Also, need to clear _inbuf after each parsing.
// XXX Switch the reads (at least) to be asynchronous transfers.
// XXX Consider querying/parsing the position rather than just the status.
// XXX Make sure that we get notified if the user moves the stage with the knobs.
// XXX Check for running into the limits.

#include "vrpn_LUDL.h"
#include <string.h>

#define	REPORT_ERROR(msg)	{ send_text_message(msg, timestamp, vrpn_TEXT_ERROR) ; if (d_connection && d_connection->connected()) d_connection->send_pending_reports(); }

#if defined(VRPN_USE_LIBUSB_1_0)

// USB vendor and product IDs for the models we support
static const vrpn_uint16 LUDL_VENDOR = 0x6969;
static const vrpn_uint16 LUDL_USBMAC6000 = 0x1235;


vrpn_LUDL_USBMAC6000::vrpn_LUDL_USBMAC6000(const char *name, vrpn_Connection *c, bool do_recenter)
  : vrpn_Analog(name, c)
  , vrpn_Analog_Output(name, c)
  , _device_handle(NULL)
  , _incount(0)
  , _endpoint(2)  // All communications is with Endpoint 2 for the MAC6000
{
  // Open and claim a device with the expected vendor and product ID.
  if ( libusb_init(&_context) != 0) {
    fprintf(stderr,"vrpn_LUDL_USBMAC6000: can't init LibUSB\n");
    return;
  }
  //printf("dbg: Opening device\n");
  if ( (_device_handle = libusb_open_device_with_vid_pid(_context, LUDL_VENDOR, LUDL_USBMAC6000)) == NULL) {
    fprintf(stderr,"vrpn_LUDL_USBMAC6000: can't find any USBMac6000 devices\n");
#ifdef  _WIN32
    fprintf(stderr,"                      (Did you install a Zadig.exe or other LibUSB-compatible driver?)\n");
#endif
#ifdef linux
    fprintf(stderr,"                      (Did you remember to run as root?)\n");
#endif
    return;
  }
  //printf("dbg: Claiming interface\n");
  if ( libusb_claim_interface(_device_handle, 0) != 0) {
    fprintf(stderr,"vrpn_LUDL_USBMAC6000: can't claim interface for this device\n");
#ifdef linux
    fprintf(stderr,"                      (Did you remember to run as root?)\n");
#endif
    libusb_close(_device_handle);
    _device_handle = NULL;
    libusb_exit(_context);
    _context = NULL;
    return;
  }

  // Initialize our analog values.
  vrpn_Analog::num_channel = 2;
  memset(channel, 0, sizeof(channel));
  memset(last, 0, sizeof(last));

  // Initialize our analog output values
  vrpn_Analog_Output::o_num_channel = 2;
  memset(o_channel, 0, sizeof(o_channel));

  // Recenter if we have been asked to.  This takes a long time, during which the
  // constructor is locked up.
  if (do_recenter) {
    //printf("dbg: Recentering\n");
    recenter();
  }

  // Register to receive the message to request changes and to receive connection
  // messages.
  if (d_connection != NULL) {
    if (register_autodeleted_handler(request_m_id, handle_request_message,
      this, d_sender_id)) {
	  fprintf(stderr,"vrpn_LUDL_USBMAC6000: can't register handler\n");
	  d_connection = NULL;
    }
    if (register_autodeleted_handler(request_channels_m_id, handle_request_channels_message,
      this, d_sender_id)) {
	  fprintf(stderr,"vrpn_LUDL_USBMAC6000: can't register handler\n");
	  d_connection = NULL;
    }
    if (register_autodeleted_handler(d_ping_message_id, handle_connect_message,
      this, d_sender_id)) {
	  fprintf(stderr,"vrpn_LUDL_USBMAC6000: can't register handler\n");
	  d_connection = NULL;
    }
  } else {
	  fprintf(stderr,"vrpn_LUDL_USBMAC6000: Can't get connection!\n");
  }

  // Allocate space to store the axis status and record that the axes are stopped.
  if ( (_axis_moving = new bool[vrpn_Analog::num_channel]) == NULL) {
    fprintf(stderr,"vrpn_LUDL_USBMAC6000: Out of memory\n");
  }
  if ( (_axis_destination = new vrpn_float64[vrpn_Analog::num_channel]) == NULL) {
    fprintf(stderr,"vrpn_LUDL_USBMAC6000: Out of memory\n");
  }
  int i;
  for (i = 0; i < vrpn_Analog::num_channel; i++) {
    _axis_moving[i] = false;
  }
}

vrpn_LUDL_USBMAC6000::~vrpn_LUDL_USBMAC6000()
{
  if (_device_handle) {
    libusb_close(_device_handle);
    _device_handle = NULL;
  }
  if (_context) {
    libusb_exit(_context);
    _context = NULL;
  }

  // Get rid of the arrays we allocated in the constructor
  if (_axis_moving != NULL) { delete [] _axis_moving; _axis_moving = NULL; }
  if (_axis_destination != NULL) { delete [] _axis_destination; _axis_destination = NULL; }
}

bool vrpn_LUDL_USBMAC6000::check_for_data(void)
{
  if (_device_handle == NULL) {
    return false;
  }

  // Let libusb handle any outstanding events
  struct timeval	zerotime;
  zerotime.tv_sec = 0;
  zerotime.tv_usec = 0;
  libusb_handle_events_timeout(_context, &zerotime);

  // Try to read as many characters as are left in the buffer from
  // the device.  Keep track of how many we get.

  int chars_to_read = _INBUFFER_SIZE - _incount;
  int chars_read = 0;
  int ret;
  //printf("dbg: Starting bulk receive\n");
  ret = libusb_bulk_transfer(_device_handle, _endpoint | LIBUSB_ENDPOINT_IN,
    &_inbuffer[_incount], chars_to_read, &chars_read, 1);
  //printf("dbg: Finished bulk receive\n");
  if ( (ret != LIBUSB_SUCCESS) && (ret != LIBUSB_ERROR_TIMEOUT) ) {
#ifdef libusb_strerror
    fprintf(stderr, "vrpn_LUDL_USBMAC6000::check_for_data(): Could not read data: %s\n",
      libusb_strerror(static_cast<libusb_error>(ret)));
#else
    fprintf(stderr, "vrpn_LUDL_USBMAC6000::check_for_data(): Could not read data: code %d\n",
      ret);
#endif

    return false;
  }
  _incount += chars_read;
  return true;
}


void vrpn_LUDL_USBMAC6000::mainloop()
{
  if (_device_handle == NULL) {
    return;
  }

  // Let libusb handle any outstanding events
  struct timeval	zerotime;
  zerotime.tv_sec = 0;
  zerotime.tv_usec = 0;
  libusb_handle_events_timeout(_context, &zerotime);

  // If one of the axes is moving, check to see whether it has stopped.
  // If so, report its new position.
  // XXX Would like to change this to poll (or have the device send
  // continuously) the actual position, rather than relying on it having
  // gotten where we asked it to go).
  if (!_axis_moving || !_axis_destination) { return; }
  int i;
  for (i = 0; i < vrpn_Analog::num_channel; i++) {
    if (_axis_moving[i]) {
      if (!ludl_axis_moving(i+1)) {
        vrpn_Analog::channel[i] = _axis_destination[i];
        _axis_moving[i] = false;
      }
    }
  }

  // Let all of the servers do their thing.
  server_mainloop();
  vrpn_gettimeofday(&_timestamp, NULL);
  report_changes();

  vrpn_Analog_Output::server_mainloop();
  vrpn_Analog::server_mainloop();
}

void vrpn_LUDL_USBMAC6000::report(vrpn_uint32 class_of_service) {
	vrpn_Analog::timestamp = _timestamp;
	vrpn_Analog::report(class_of_service);
}

void vrpn_LUDL_USBMAC6000::report_changes(vrpn_uint32 class_of_service) {
	vrpn_Analog::timestamp = _timestamp;
	vrpn_Analog::report_changes(class_of_service);
}

void vrpn_LUDL_USBMAC6000::flush_input_from_ludl(void)
{
  // Clear the input buffer, read all available characters
  // from the endpoint we're supposed to use, then clear it
  // again -- throwing away all data that was coming from the device.
  _incount = 0;
  check_for_data();
  _incount = 0;
}

// Construct a command using the specific language of the USBMAC6000
// and send the message to the device.
// I got the format for this message from the code in USBMAC6000.cpp from
// the video project, as implemented by Ryan Schubert at UNC.
bool vrpn_LUDL_USBMAC6000::send_usbmac_command(unsigned device, unsigned command, unsigned index, int value)
{
  if (_device_handle == NULL) {
    return false;
  }

  // Let libusb handle any outstanding events
  struct timeval	zerotime;
  zerotime.tv_sec = 0;
  zerotime.tv_usec = 0;
  libusb_handle_events_timeout(_context, &zerotime);

  char msg[1024];
  sprintf(msg, "can %u %u %u %i\n", device, command, index, value);
  int len = strlen(msg);
  int sent_len = 0;
  msg[len-1] = 0xD;

  //printf("dbg: Starting bulk send command\n");
  int ret = libusb_bulk_transfer(_device_handle, _endpoint | LIBUSB_ENDPOINT_OUT,
            static_cast<vrpn_uint8 *>(static_cast<void*>(msg)),
            len, &sent_len, 50);
  //printf("dbg: Finished bulk send command\n");
  if ((ret != 0) || (sent_len != len)) {
#ifdef libusb_strerror
    fprintf(stderr,"vrpn_LUDL_USBMAC6000::send_usbmac_command(): Could not send: %s\n",
      libusb_strerror(static_cast<libusb_error>(ret)));
#else
    fprintf(stderr,"vrpn_LUDL_USBMAC6000::send_usbmac_command(): Could not send: code %d\n",
      ret);
#endif
    return false;
  }
  return true;
}

// Interpret one response from the device and place its resulting value
// in the return parameter.
// I got the format for this message from the code in USBMAC6000.cpp from
// the video project, as implemented by Ryan Schubert at UNC.
bool vrpn_LUDL_USBMAC6000::interpret_usbmac_ascii_response(const vrpn_uint8 *buffer, int *value_return)
{
  if (buffer == NULL) { return false; }
  const char *charbuf = static_cast<const char *>(static_cast<const void *>(buffer));

  char acolon[32], can[32];
  int device = 0, command = 0, index = 0, value = 0;
  if (sscanf(charbuf, "%s  %s %i %i %i %i", acolon, can, &device, &command, &index, &value) <= 0) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::interpret_usbmac_ascii_response(): Could not parse response");
    return false;
  }

  *value_return = value;
  return true;
}

// I got the algorithm for recentering from the code in USBMAC6000.cpp from
// the video project, as implemented by Ryan Schubert at UNC.
// XXX This takes a LONG TIME to finish and relies on messages coming back;
// consider making it asynchronous.
bool vrpn_LUDL_USBMAC6000::recenter(void)
{
  // Send the command to make the X axis go to both ends of its
  // range and then move into the center.
  if (!send_usbmac_command(1, 65, 7, 100000)) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::recenter(): Could not send command 1");
    return false;
  }
  printf("vrpn_LUDL_USBMAC6000::recenter(): Waiting for X-axis center\n");
  vrpn_SleepMsecs(500); // XXX Why sleep?

  // Let libusb handle any outstanding events
  struct timeval	zerotime;
  zerotime.tv_sec = 0;
  zerotime.tv_usec = 0;
  libusb_handle_events_timeout(_context, &zerotime);

  flush_input_from_ludl();
  while(ludl_axis_moving(1)) {
    vrpn_SleepMsecs(10);
    libusb_handle_events_timeout(_context, &zerotime);
  }

  // Send the command to record the value at the center of the X axis as
  // 694576 ticks (XXX magic number from where?)
  if (!send_usbmac_command(1, 83, 5, 694576)) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::recenter(): Could not send command 2");
    return false;
  }
  channel[0] = 694576;

  // Send the command to make the Y axis go to both ends of its
  // range and then move into the center.
  if (!send_usbmac_command(2, 65, 7, 100000)) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::recenter(): Could not send command 3");
    return false;
  }
  printf("vrpn_LUDL_USBMAC6000::recenter(): Waiting for Y-axis center\n");
  vrpn_SleepMsecs(500); // XXX Why sleep?

  // Let libusb handle any outstanding events
  libusb_handle_events_timeout(_context, &zerotime);

  flush_input_from_ludl();
  while(ludl_axis_moving(2)) {
    vrpn_SleepMsecs(10);
    libusb_handle_events_timeout(_context, &zerotime);
  }

  // Send the command to record the value at the center of the Y axis as
  // 1124201 ticks (XXX magic number from where?)
  if (!send_usbmac_command(2, 83, 5, 1124201)) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::recenter(): Could not send command 4");
    return false;
  }
  channel[1] = 1124201;

  return true;
}

// I got the algorithm for checking if the axis is moving
// from the code in USBMAC6000.cpp from
// the video project, as implemented by Ryan Schubert at UNC.
// The first axis is 1 in this function.
bool vrpn_LUDL_USBMAC6000::ludl_axis_moving(unsigned axis)
{
  // Request the status of the axis.  In particular, we look at the
  // bits telling whether each axis is busy.
  flush_input_from_ludl();
  if (!send_usbmac_command(32, 84, 63, 0)) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::ludl_axis_moving(): Could not send command 1");
    return false;
  }

  // Read from the device to find the status.  We call the check_for_data() method
  // to look for a response.
  unsigned watchdog = 0;
  while (_incount == 0) {
    if (!check_for_data()) {
      REPORT_ERROR("vrpn_LUDL_USBMAC6000::ludl_axis_moving(): Could not get report");
      return false;
    }

    // If it has been too long, re-send the request to the stage.
    // XXX We should not be losing characters... figure out what is causing us to
    // have to resend.
    if (++watchdog == 25) {  // 25 ms (timeout is 1ms)
      if (!send_usbmac_command(32, 84, 63, 0)) {
        REPORT_ERROR("vrpn_LUDL_USBMAC6000::ludl_axis_moving(): Could not resend command 1");
        return false;
      }
      watchdog = 0;
    }
  }
  int status = 0;
  if (!interpret_usbmac_ascii_response(_inbuffer, &status)) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::ludl_axis_moving(): Could not parse report");
    return false;
  }
  _incount = 0; // XXX Should parse more than one report if there is one.
  int axisMaskBit = 0x0001 << axis;
  return (status & axisMaskBit) != 0;
}

bool vrpn_LUDL_USBMAC6000::move_axis_to_position(int axis, int position)
{
  if (!_device_handle) { return false; }
  if (!_axis_destination || !_axis_moving) { return false; }

  // Send the command to the device asking it to move.
  // command: 65 - MOTOR_ACTION
  // index:   0  - START_MOTOR_TARGET
  if (!send_usbmac_command(axis, 65, 0, position)) {
    REPORT_ERROR("vrpn_LUDL_USBMAC6000::move_axis_to_position(): Could not send command");
    return false;
  }

  // Indicate that we're expecting this axis to be moving and where we think it is
  // going, so that when the axis becomes no longer busy we know that we have gotten
  // there.

  _axis_destination[axis-1] = position;
  _axis_moving[axis-1] = true;
  return true;
}

int vrpn_LUDL_USBMAC6000::handle_request_message(void *userdata, vrpn_HANDLERPARAM p)
{
    const char	  *bufptr = p.buffer;
    vrpn_int32	  chan_num;
    vrpn_int32	  pad;
    vrpn_float64  value;
    vrpn_LUDL_USBMAC6000 *me = (vrpn_LUDL_USBMAC6000 *)userdata;

    // Read the parameters from the buffer
    vrpn_unbuffer(&bufptr, &chan_num);
    vrpn_unbuffer(&bufptr, &pad);
    vrpn_unbuffer(&bufptr, &value);

    // Set the position to the appropriate value, if the channel number is in the
    // range of the ones we have.
    if ( (chan_num < 0) || (chan_num >= me->o_num_channel) ) {
      char msg[1024];
      sprintf(msg,"vrpn_LUDL_USBMAC6000::handle_request_message(): Index out of bounds (%d of %d), value %lg\n",
	chan_num, me->o_num_channel, value);
      me->send_text_message(msg, me->timestamp, vrpn_TEXT_ERROR);
      return 0;
    }

    me->move_axis_to_position(chan_num + 1, static_cast<int>(value));
    return 0;
}

int vrpn_LUDL_USBMAC6000::handle_request_channels_message(void* userdata, vrpn_HANDLERPARAM p)
{
    int i;
    const char* bufptr = p.buffer;
    vrpn_int32 num;
    vrpn_int32 pad;
    vrpn_LUDL_USBMAC6000* me = (vrpn_LUDL_USBMAC6000 *)userdata;

    // Read the values from the buffer
    vrpn_unbuffer(&bufptr, &num);
    vrpn_unbuffer(&bufptr, &pad);
    if (num > me->o_num_channel) {
      char msg[1024];
      sprintf(msg,"vrpn_LUDL_USBMAC6000::handle_request_channels_message(): Index out of bounds (%d of %d), clipping\n",
	num, me->o_num_channel);
      me->send_text_message(msg, me->timestamp, vrpn_TEXT_ERROR);
      num = me->o_num_channel;
    }
    for (i = 0; i < num; i++) {
        vrpn_unbuffer(&bufptr, &(me->o_channel[i]));
        me->move_axis_to_position(i + 1, static_cast<int>(me->o_channel[i]));
    }

    return 0;
}

/** When we get a connection request from a remote object, send our state so
    they will know it to start with. */
int vrpn_LUDL_USBMAC6000::handle_connect_message(void *userdata, vrpn_HANDLERPARAM)
{
    vrpn_LUDL_USBMAC6000 *me = (vrpn_LUDL_USBMAC6000 *)userdata;

    me->report(vrpn_CONNECTION_RELIABLE);
    return 0;
}

// End of VRPN_USE_HID
#endif
