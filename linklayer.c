/* Functions to implement link layer protocol - full versions
	with acknowledgements and re-transmission:
   LL_connect() connects to another computer;
   LL_discon()  disconnects;
   LL_send()    sends a block of data;
   LL_receive() waits to receive a block of data.
   All functions take a debug argument - if non-zero, they print
   messages explaining what is happening.  Regardless of debug,
   functions print messages on errors.
   LL_send and LL_receive behave in a simpler way if debug is 1,
   to facilitate testing of some parts of the protocol.
   All functions return negative values on error or failure.
   Definitions of constants are in the header file.  */

typedef unsigned char byte;  // byte is an 8-bit value, range 0 to 255

#include <stdio.h>      // input-output library: print & file operations
#include <time.h>       // for timing functions
#include "physical.h"   // physical layer functions
#include "linklayer.h"  // these functions

/* These variables need to retain their values between function calls, so they
   are declared as static.  By declaring them outside any function, they are
   made available to all the functions in this file.  */
static int seqNumTx;        // transmit block sequence number
static int lastSeqRx;       // sequence number of last good block received
static int connected = FALSE;   // keep track of state of connection
static int framesSent = 0;  // count of frames sent
static int acksSent = 0;    // count of ACKs sent
static int naksSent = 0;    // count of NAKs sent
static int acksRx = 0;      // count of ACKs received
static int naksRx = 0;      // count of NAKs received
static int badFrames = 0;   // count of bad frames received
static int goodFrames = 0;  // count of good frames received
static int timeouts = 0;    // count of timeouts
static long timerRx;        // time value for timeouts at receiver
static long connectTime;    // time when connection established

// ===========================================================================
/* Function to connect to another computer.
   It just calls PHY_open() and reports any error.
   It also initialises counters for debug purposes.  */
int LL_connect(int debug)
{
    // Try to connect - set suitable parameters here...
    int retCode = PHY_open(PORTNUM,BIT_RATE,8,PARITYNONE,1000,50,PROB_ERR);
    if (retCode == SUCCESS)   // check if succeeded
    {
        connected = TRUE;   // record that we are connected
        seqNumTx = 0;       // set first sequence number for sender
        lastSeqRx = -1;     // set impossible value for last seq. received
        framesSent = 0;     // initialise counters for report
        acksSent = 0;
        naksSent = 0;
        acksRx = 0;
        naksRx = 0;
        badFrames = 0;
        goodFrames = 0;
        timeouts = 0;
        connectTime = clock();  // capture time when connection established
        if (debug) printf("LL: Connected\n");
        return SUCCESS;
    }
    else  // failed
    {
        connected = FALSE;  // record lack of connection
        printf("LL: Failed to connect, PHY returned code %d\n",retCode);
        return -retCode;  // return negative error code
    }
}


// ===========================================================================
/* Function to disconnect from other computer.
   It just calls PHY_close() and prints debug info.  */
int LL_discon(int debug)
{
    long elapsedTime = clock() - connectTime;  // measure time connected
    float connTime = ((float) elapsedTime ) / CLOCKS_PER_SEC; // convert to seconds
    int retCode = PHY_close();  // try to disconnect
    connected = FALSE;  // assume no longer connected
    if (retCode == SUCCESS)   // check if succeeded
    {
        if (debug) // print all counters (don't know if were sending or receiving)
        {
            printf("\nLL: Disconnected after %.2f s.  Sent %d data frames\n",
                   connTime, framesSent);
            printf("LL: Received %d good and %d bad frames, had %d timeouts\n",
                   goodFrames, badFrames, timeouts);
            printf("LL: Sent %d ACKs, %d NAKs\n", acksSent, naksSent);
            printf("LL: Received %d ACKs, %d NAKs\n", acksRx, naksRx);
        }
        return SUCCESS;

    }
    else  // failed
    {
        printf("LL: Failed to disconnect, PHY returned code %d\n", retCode);
        return -retCode;  // return negative error code
    }
}


// ===========================================================================
/* Function to send a block of data in a frame.
   Arguments:  dataTx is a pointer to an array of data of bytes,
               nData is the number of data bytes to send,
               debug sets the mode of operation and controls printing.
   The return value indicates success or failure.
   If connected, builds a frame, then sends the frame using PHY_send.
   If debug is 1, it regards this as success, and returns.
   Otherwise, it waits for a reply, up to a time limit.
   What happens after that is for you to decide...  */
int LL_send(byte *dataTx, int nData, int debug)
{
    static byte frameTx[3*MAX_BLK];  // array large enough for frame
    static byte frameAck[2*ACK_SIZE]; // twice expected ack frame size
    int nFrame = 0;         // size of frame
    int sizeAck = 0;        // size of ACK frame received
    int seqAck;             // sequence number in response received
    int attempts = 0;       // number of attempts to send
    int success = 0;        // flag to indicate block sent and ACKed
    int retVal;             // return value from other functions

    // First check if connected
    if (connected == FALSE)
    {
        printf("LLS: Attempt to send while not connected\n");
        return BADUSE;  // error code
    }

    // Then check if block size OK - adjust limit for your design
    if (nData > MAX_BLK)
    {
        printf("LLS: Cannot send block of %d bytes, max %d\n", nData, MAX_BLK);
        return BADUSE;  // error code
    }

    // Build the frame
    nFrame = buildDataFrame(frameTx, dataTx, nData, seqNumTx);

    // Then loop, sending the frame and waiting for response
    do
    {

        // Send the frame, then check for problems
        retVal = PHY_send(frameTx, nFrame);  // send frame bytes
        if (retVal != nFrame)  // problem!
        {
            printf("LLS: Block %d, failed to send frame\n", seqNumTx);
            return FAILURE;  // error code
        }

        framesSent++;  // increment frame counter (for debug)
        attempts++;    // increment attempt counter, so don't try forever
        if (debug) printf("LLS: Sent frame %d bytes, block %d, attempt %d\n",
                          nFrame, seqNumTx, attempts);

        // In simple mode, this is all we have to do (there are no acks)
        if (debug == SIMPLE)
        {
            success = 1;    // set success to 1 to end the loop
            continue;       // and go straight to the while statement
        }

        // Otherwise, we must wait to receive a response (ack or nak)
        sizeAck = getFrame(frameAck, 2*ACK_SIZE, TX_WAIT);
        if (sizeAck < 0)  // some problem receiving
        {
            return FAILURE;  // quit if error
        }
        if (sizeAck == 0)  // timeout
        {
            if (debug) printf("LLS: Timeout waiting for response\n");
            timeouts++;  // increment counter for report
            // What else should be done about that (if anything)?
            // If success is not set to 1, this loop will continue,
            // so will re-transmit the frame and wait for a response...
        }
        else  // we have received a frame - check it
        {
            if (checkFrame(frameAck, sizeAck) == FRAMEGOOD)  // good frame
            {
                goodFrames++;  // increment counter for report
                seqAck = frameAck[SEQNUMPOS]; // extract sequence number
                // Need to check if this is a positive ACK,
                // and if it relates to the data block just sent...
                if ( 1 )  // need a sensible test here!!
                {
                    if (debug) printf("LLS: ACK received, seq %d\n", seqAck);
                    acksRx++;       // increment counter for report
                    success = 1;    // job is done
                }
                else // could be NAK, or ACK for wrong block...
                {
                    if (debug) printf("LLS: Response received, type %d, seq %d\n",
                            99, 99 );  // need sensible values here!!
                    naksRx++;          // increment counter for report
                    // What else should be done about that (if anything)?
                }
            }
            else  // bad frame received - failed error check
            {
                badFrames++;  // increment counter for report
                if (debug) printf("LLS: Bad frame received\n");
                // What should be done about that (if anything)?
            }

        }  // end of received frame processing
    }
    while (!success && attempts < MAX_TRIES);  // repeat until succeed or reach limit

    if (success)  // data block has been sent and acknowledged
    {
        seqNumTx = next(seqNumTx);  // increment sequence number
        return SUCCESS;
    }
    else    // maximum number of attempts has been reached, without success
    {
        if (debug) printf("LLS: Block %d, tried %d times, failed\n",
                          seqNumTx, MAX_TRIES);
        return GIVEUP;  // tried enough times, giving up
    }

}  // end of LL_send


// ===========================================================================
/* Function to receive a frame and extract a block of data.
   Arguments:  dataRx is a pointer to an array to hold the data block,
               maxdata is the max size of the data block,
               debug sets the mode of operation and controls printing.
   The return value is the size of the data block, or negative on error.
   If connected, tries to get a frame from received bytes.
   If a frame is found, check if it is a good frame, with no errors.
   In simple mode, data is extracted from a good frame, and the function returns.
   For bad frames, a block of 10 # characters is returned as the data.
   In normal mode, the sequence number is also checked, and the function
   loops until it gets a good frame with the expected sequence number,
   then returns with the data bytes from the frame.  */
int  LL_receive(byte *dataRx, int maxData, int debug)
{
    static byte frameRx[3*MAX_BLK];  // create array to hold frame
    int nData = 0;   // number of data bytes received
    int nFrame = 0;  // number of bytes in frame received
    int seqNumRx = 0;  // sequence number of received frame
    int success = 0;   // flag to indicate success
    int attempts = 0;  // attempt counter
    int i = 0;         // used in for loop
    int expected = next(lastSeqRx);  // calculate expected sequence number

    // First check if connected
    if (connected == FALSE)
    {
        printf("LLR: Attempt to receive while not connected\n");
        return BADUSE;  // error code
    }

    /* Loop to receive a frame, repeats until a frame is received.
       In normal mode, repeats until a good frame with the expected
       sequence number is received. */
    do
    {
        // First get a frame, up to maximum size of frame array.
        // Function returns number of bytes in frame, or negative if error
        nFrame = getFrame(frameRx, 3*MAX_BLK, RX_WAIT);
        if (nFrame < 0)  // some problem receiving
        {
            return FAILURE;  // quit if error
        }

        attempts++;  // increment attempt counter
        if (nFrame == 0)  // timeout
        {
            printf("LLR: Timeout trying to receive frame, attempt %d\n",
								attempts);
            // nothing received, so no response needed
            // If success is not set to 1, loop will continue and try again...
        }
        else  // we have received a frame
        {
            if (debug) printf("LLR: Got frame, %d bytes, attempt %d\n",
								        nFrame, attempts);

            // Next step is to check it for errors
            if (checkFrame(frameRx, nFrame) == FRAMEBAD ) // frame is bad
            {
                badFrames++;  // increment bad frame counter
                if (debug) printf("LLR: Bad frame received\n");
                if (debug) printFrame(frameRx, nFrame);
                // In simple mode, just return some dummy data
                if (debug == SIMPLE)  // simple mode
                {
                    success = 1;  // set the flag to indicate success
                    // put some dummy bytes in the data array
                    for (i=0; i<10; i++) dataRx[i] = 35; // # symbol
                    nData = 10;     // number of dummy bytes
                }
                else
                {
                // In normal mode, this is not a success!
                // Maybe send a response to the sender ?
                // if so, what sequence number ?
                }

            }
            else  // we have a good frame - process it
            {
                goodFrames++;  // increment good frame counter
                nData = processFrame(frameRx, nFrame, dataRx, maxData, &seqNumRx);
                if (debug) printf("LLR: Received block %d with %d data bytes\n",
                                  seqNumRx, nData);

                // In simple mode, just accept the data - no further checking
                if (debug == SIMPLE) success = 1;
                // In normal mode, need to check the sequence number
                else if (seqNumRx == expected)  // expected data block
                {
                    success = 1;  // job is done
                    lastSeqRx = seqNumRx;  // update last sequence number
                    // maybe send a response to the sender ?
                    // if so, what sequence number ?

                }
                else if (seqNumRx == lastSeqRx) // duplicate data block
                {
                    if (debug) printf("LLR: Duplicate rx seq. %d, expected %d\n",
                                  seqNumRx, expected);
                    // what should be done about this?

                }
                else // some other data block??
                {
                    if (debug) printf("LLR: Unexpected block rx seq. %d, expected %d\n",
                                  seqNumRx, expected);
                    // what should be done about this?

                }  // end of sequence number checking

            }  // end of good frame processing

        } // end of received frame processing
    }
    while (!success && attempts < MAX_TRIES);

    if (success)  // have received good frame with expected sequence number
        return nData;  // return number of data bytes extracted from frame
    else // failed to get a good frame within limits
        return GIVEUP;  // this is an error situation - give up

}  // end of LL_receive


// ===========================================================================
/* Function to build a frame from a block of data.
   This function puts the header bytes into the frame,
   then copies in the data bytes.  Then it adds the trailer bytes.
   It calculates the total number of bytes in the frame,
   and returns this value to the calling function.
   Arguments: frameTx is a pointer to an array to hold the frame,
              dataTx is the array of data bytes to be put in the frame,
              nData is the number of data bytes to be put in the frame,
              seq is the sequence number to include in the frame header.
   The return value is the total number of bytes in the frame.  */
int buildDataFrame(byte *frameTx, byte *dataTx, int nData, int seq)
{
    int i = 0;  // for use in loop
    int checksum = 0; // initialise checksum
    int nByte=HEADERSIZE; // initialize bByte
    // Build the header
    frameTx[0] = STARTBYTE;  // start of frame marker byte
    frameTx[SEQNUMPOS] = (byte) seq;  // sequence number as given

    // Copy the data bytes into the frame & stuff bytes into the frame
    for (i = 0; i < nData; i++)
    {
        if (special(dataTx[i]))
        frameTx[nByte++] = STUFFBYTE;
        frameTx[nByte++] = dataTx[i];  // copy the data byte, advance the counter

        checksum += dataTx[i];

    }
    checksum %= CSMOD;


    // Build the trailer - just an end marker for now
    if (special(checksum))
    {frameTx[nByte++] = STUFFBYTE;}
    frameTx[nByte++] =   checksum; // checkSum byte

    frameTx[nByte++] =   ENDBYTE; // end of frame marker byte

    // Return the size of the frame
    return nByte;
}


// ===========================================================================
/* Function to find and extract a frame from the received bytes.
   Arguments: frameRx is a pointer to an array of bytes to hold the frame,
              maxSize is the maximum number of bytes to receive,
              timeLimit is the time limit for receiving a frame.
   The return value is the number of bytes in the frame, or negative if error. */

int getFrame(byte *frameRx, int maxSize, float timeLimit)
{
    int nRx = 0;  // number of bytes received so far
    int retVal = 0;  // return value from other functions
    int stuff_flag=0; //initialize stuff flag
    int marker_flag=0;//initialize marker flag

    /*do {
        get one byte from physical layer;
        if (stuff_flag == 1) stuff_flag = 0;  // previous byte was STUFF
        else if (this_byte == MARKER) marker_flag = 1;  // marker has been found
        else if (this_byte == STUFF) stuff_flag = 1;  // stuff byte has been found
    } while ( (marker_flag == 0) && !timeUp() );  // repeat until marker found or time up*/
    timerRx = timeSet(timeLimit);  // set time limit to wait for frame

    // First search for the start of frame marker
  /*  do
    {
        retVal = PHY_get((frameRx+nRx), 1); // get one byte at a time
        nRx+=1;
        // Return value is number of bytes received, or negative for error
        if (retVal < 0) return retVal;  // check for error and give up
        if (stuff_flag == 1) stuff_flag = 0;  // previous byte was STUFF
        else if (frameRx[nRx-1] == STARTBYTE)  marker_flag = 1;  // marker has been found
        else if (frameRx[nRx-1] == STUFFBYTE)  stuff_flag = 1;  // stuff byte has been found
     }
    while ((marker_flag==0) && !timeUp(timerRx)); */
        do
    {
        retVal = PHY_get(frameRx, 1); // get one byte at a time
        // Return value is number of bytes received, or negative for error
        if (retVal < 0) return retVal;  // check for error and give up
     }
    while (((retVal < 1) || (frameRx[0] != STARTBYTE)) && !timeUp(timerRx));
    // until we get a byte which is a start of frame marker, or timeout

    // If we are out of time, return 0 - no useful bytes received
    if (timeUp(timerRx))
    {
        printf("LLGF: Timeout seeking START, %d bytes received\n", nRx);
        return 0;  // no bytes received, but not an error situation
    }
    nRx = 1;  // if there was no timeout, we got 1 byte

    /* Now collect more bytes, until we find the end marker.
       If we had a byte count, we could do this more efficiently, by
       asking PHY_get() to collect all the remaining bytes in one call. */
    do
    {
        retVal = PHY_get((frameRx + nRx), 1);  // get one byte at a time
        if (retVal < 0) return retVal;  // check for error and give up
        else nRx += retVal;  // update the bytes received count

        //discard the stuffed bytes
        if (stuff_flag == 1) stuff_flag = 0;  // previous byte was STUFF
        else if (frameRx[nRx-1] == STARTBYTE || frameRx[nRx-1] == ENDBYTE) marker_flag = 1;  // marker has been found
        else if (frameRx[nRx-1] == STUFFBYTE)  // stuff byte has been found
        {
              stuff_flag = 1;  // set the flag to remember this
              nRx--;     // decrement nRx, so stuff byte will be over-written
        }} while ((marker_flag == 0) && !timeUp(timerRx));
    // until we get the end of frame marker or timeout

    // If time up, no end marker, so bad frame, return 0
    if (timeUp(timerRx))
    {
        printf("LLGF: Timeout seeking END, %d bytes received\n", nRx);
        return 0;  // no bytes received, but not an error situation
    }

    // Otherwise, we found the end marker
    return nRx;  // return the number of bytes in the frame
}
  // end of getFrame


// ===========================================================================
/* Function to check a received frame for errors.
   Arguments: frameRx is a pointer to an array of bytes holding a frame,
              nFrame is the number of bytes in the frame.
   As a minimum, this function should check the error detecting code.
   This example just checks the start and end markers.
   Return value indicates if the frame is good or bad.   */
int checkFrame(byte *frameRx, int nFrame)
{    int checksum = 0; // initialise checksum
     int i;
    // Check the start-of-frame marker in the first byte
    /*if (frameRx[0] != STARTBYTE)
    {
        printf("LLCF: Frame bad - no start marker\n");
        return FRAMEBAD;
    }

    // Check the end-of-frame marker in the last byte
    if (frameRx[nFrame-1] != ENDBYTE)
    {
        printf("LLCF: Frame bad - no end marker\n");
        return FRAMEBAD;
    } */
    // Need to check the error detecting code...
    // If this check fails, print a message and return FRAMEBAD
    for (i = HEADERSIZE; i < nFrame-TRAILERSIZE ; i++)
    {
        checksum += frameRx[i];

    }
    checksum %= CSMOD;


    if  (frameRx[nFrame-TRAILERSIZE] !=checksum)
    {
        printf("LLCF:  Frame bad - checksum doesn't match\n");
        return FRAMEBAD;
    }


    // If all tests are passed, indicate a good frame
    return FRAMEGOOD;
}  // end of checkFrame


// ===========================================================================
/* Function to process a received frame, to extract the data & sequence number.
   The frame has already been checked for errors, so this simple
   implementation assumes everything is where is should be.
   Arguments: frameRx is a pointer to the array holding the frame,
              nFrame is the number of bytes in the frame,
              dataRx is a pointer to an array to hold the data bytes,
              maxData is the max number of data bytes to extract,
              seqNum is a pointer to the sequence number.
   The return value is the number of data bytes extracted. */
int processFrame(byte *frameRx, int nFrame,
                 byte *dataRx, int maxData, int *seqNum)
{
    int i = 0;  // for use in loop
    int nData;  // number of data bytes in the frame

    // First get the sequence number from its place in the header
    *seqNum = frameRx[SEQNUMPOS];

    // Calculate the number of data bytes, based on frame size
    nData = nFrame - HEADERSIZE - TRAILERSIZE;
    if (nData > maxData) nData = maxData;  // limit to the max allowed

    // Now copy the data bytes from the middle of the frame
    for (i = 0; i < nData; i++)
    {
        dataRx[i] = frameRx[HEADERSIZE + i];  // copy one byte
    }

    return nData;  // return the size of the data block extracted
}  // end of processFrame


// ===========================================================================
/* Function to send an acknowledgement - positive or negative.
   Arguments: type is the type of acknowledgement to send,
              seq is the sequence number that the ack should carry,
              debug controls printing of messages.
   Return value indicates success or failure.
   Note type is used to update statistics for report, so the argument
   is needed even if its value is not included in the ack frame. */
int sendAck(int type, int seq, int debug)
{
    byte ackFrame[2*ACK_SIZE];  // twice expected frame size, for byte stuff
    int sizeAck = 0; // number of bytes in the ack frame so far
    int retVal; // return value from functions

    // First build the frame
    ackFrame[0] = STARTBYTE;

	// Add more bytes to the frame, and update sizeAck

    // Then send the frame and check for problems
    retVal = PHY_send(ackFrame, sizeAck);  // send the frame
    if (retVal != sizeAck)  // problem!
    {
        printf("LLSA: Failed to send response, seq. %d\n", seq);
        return FAILURE;  // error code
    }
    else  // success - update the statistics counters for report
    {
        if (type == POSACK)  acksSent++;
        else if (type == NEGACK) naksSent++;
        if (debug)
            printf("LLSA: Sent response of %d bytes, type %d, seq %d\n",
                        sizeAck, type, seq);
        return SUCCESS;
    }
}


// ===========================================================================
/* Function to advance the sequence number,
   wrapping around at maximum value.  */
int next(int seq)
{
    return ((seq + 1) % MOD_SEQNUM);
}


// ===========================================================================
/* Function to set time limit at a point in the future.
   limit   is time limit in seconds (from now)  */
long timeSet(float limit)
{
    long timeLimit = clock() + (long)(limit * CLOCKS_PER_SEC);
    return timeLimit;
}  // end of timeSet


// ===========================================================================
/* Function to check if time limit has elapsed.
   timer  is timer variable to check
   returns TRUE if time has reached or exceeded limit,
           FALSE if time has not yet reached limit.   */
int timeUp(long timeLimit)
{
    if (clock() < timeLimit) return FALSE;  // still within limit
    else return TRUE;  // time limit has been reached or exceeded
}  // end of timeUP

// ===========================================================================
/* Function to check if a byte is one of the protocol bytes.
   argument b is a byte value to check
   returns TRUE if b is a protocol byte,
           FALSE if b is not a protocol byte.   */
int special(byte b)
{
    if(b==STARTBYTE||b==ENDBYTE||b==STUFFBYTE)
    {
        return TRUE;
    }
    return FALSE;   // no checking in this version
}

// ===========================================================================
/* Function to print bytes of a frame, in groups of 10.
   For small frames, print all the bytes,
   for larger frames, just the start and end. */
void printFrame(byte *frame, int nByte)
{
    int i, j;

    if (nByte <= 50)  // small frame - print all the bytes
    {
        for (i=0; i<nByte; i+=10)  // step in groups of 10 bytes
        {
            for (j=0; (j<10)&&(i+j<nByte); j++)
            {
                printf("%3d ", frame[i+j]);  // print as number
            }
            printf(":  ");  // separator
            for (j=0; (j<10)&&(i+j<nByte); j++)
            {
                printf("%c", frame[i+j]);   // print as character
            }
            printf("\n");   // new line
        }  // end for
    }
    else  // large frame - print start and end
    {
        for (j=0; (j<10); j++)  // first 10 bytes
            printf("%3d ", frame[j]);  // print as number
        printf(":  ");  // separator
        for (j=0; (j<10); j++)
            printf("%c", frame[j]); // print as character
        printf("\n - - -\n");   // new line, separator
        for (j=nByte-10; (j<nByte); j++)  // last 10 bytes
            printf("%3d ", frame[j]);  // print as number
        printf(":  ");  // separator
        for (j=nByte-10; (j<nByte); j++)
            printf("%c", frame[j]); // print as character
        printf("\n");   // new line
    }

}  // end of printFrame
