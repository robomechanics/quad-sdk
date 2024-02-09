/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * $Id: CANReadThread.java 7377 2020-08-07 14:40:53Z Fabrice $
 * @LastChange $Date: 2020-08-07 16:40:53 +0200 (ven., 07 août 2020) $
 * 
 * Demo Application for PCANBasic JAVA JNI Interface.
 *
 * Copyright (C) 2001-2020  PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * PCAN is a registered Trademark of PEAK-System Germany GmbH
 *
 * Author: 		 Jonathan Urban/Uwe Wilhelm/Fabrice Vergnaud
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Fabrice Vergnaud <f.vergnaud@peak-system.com>
 */
package peak.can;

import java.util.HashMap;
import java.util.Vector;

import peak.can.basic.TPCANMsg;
import peak.can.basic.TPCANTimestamp;
import peak.can.basic.IRcvEventProcessor;
import peak.can.basic.PCANBasic;
import peak.can.basic.TPCANHandle;
import peak.can.basic.TPCANMsgFD;
import peak.can.basic.TPCANStatus;
import peak.can.basic.TPCANTimestampFD;

/**
 * The CANReadThread class extends Thread class and is used to process readed CAN Messages.
 * In addition, the class provides different read mode that are "By Timer" or "By Event".
 * It is possible to read CAN Messages with its Time Stamp.
 *
 * @version 1.1
 * @LastChange $Date: 2020-08-07 16:40:53 +0200 (ven., 07 août 2020) $
 * @author Jonathan Urban/Uwe Wilhelm/Fabrice Vergnaud
 *
 * @Copyright (C) 1999-2014  PEAK-System Technik GmbH, Darmstadt
 * more Info at http://www.peak-system.com
 */
public class CANReadThread extends Thread implements IRcvEventProcessor
{
    // PCANBasic instance used to call read functions
    private PCANBasic pcanBasic;
    // Collection which stores all connected channels
    private Vector<ChannelItem> connectedChannelCollection = null;
    // Collection to store readed CAN Messages
    private HashMap<Integer, TableDataRow> dataRowCollection;
    // Used to read CAN Messages with its Time stamp
    private Boolean readTimeStamp = false;

    /**
     * @return states if timestamp is used when reading CAN messages
     */
    public Boolean getReadTimeStamp()
    {
        return readTimeStamp;
    }
    /**
     * @param useReadEx states if timestamp must be used when reading CAN messages
     */
    public void setReadTimeStamp(Boolean useReadEx)
    {
        this.readTimeStamp = useReadEx;
    }

    /**
     *
     * @param pcanbasic PCANBasic instance used to call read functions
     * @param connectedChannelCollection Reference to the collection which stores all connected channels
     * @param dataRowCollection Reference to the Collection which store readed CAN Messages
     */
    public CANReadThread(PCANBasic pcanbasic, Vector<ChannelItem> connectedChannelCollection, HashMap<Integer, TableDataRow> dataRowCollection)
    {
        this.pcanBasic = pcanbasic;
        this.dataRowCollection = dataRowCollection;
        this.connectedChannelCollection = connectedChannelCollection;
    }

    /**
     * Starts thread process
     */
    public void run()
    {
        while (true)
        {
            // Process each connected channel
        	synchronized (connectedChannelCollection) {
	            for (ChannelItem item : connectedChannelCollection)
	            {
	                if ((item != MarkAllChannelItem.getInstance()) &&
	                        (item.getWorking())) {
	                    if (item.isCanFd())
	                        callAPIFunctionReadFd(item.getHandle());
	                    else
	                        callAPIFunctionRead(item.getHandle());
	                }
	            }
        	}
            // Sleep Time
            try
            {
                Thread.sleep(10);
            }
            catch (InterruptedException e)
            {
                return;
            }
        }
    }

    /**
     * Calls the PCANBasic Read Function according the readTimeStamp parameter
     *
     * @param handle The handle of a PCAN Channel
     */
    public void callAPIFunctionRead(TPCANHandle handle)
    {
        //Local variables
        TPCANMsg canMessage = null;
        TPCANTimestamp rcvTime = null;
        TableDataRow dataRow = null;
        TPCANStatus ret;
        int messageID = 0;

        try
        {
            do
            {
                // Create new CAN Message
                canMessage = new TPCANMsg();
                // Create new TimeStamp object
                rcvTime = new TPCANTimestamp();
                //If TimeStamp is needed
                if (readTimeStamp)
                    // We execute the "Read" function of the PCANBasic
                    ret = pcanBasic.Read(handle, canMessage, rcvTime);
                //If TimeStamp is not needed
                else
                    // We execute the "Read" function of the PCANBasic
                    ret = pcanBasic.Read(handle, canMessage, null);

                //Process result
                if (ret == TPCANStatus.PCAN_ERROR_OK)
                {
                    //Gets CAN Message ID
                    messageID = canMessage.getID();
                    
                    //Critical Area: dataRowCollection is used in multiple threads
                    synchronized (Application.token)
                    {
                        // Searchs dataRowCollection contains CAN Message ID
                        if (dataRowCollection.containsKey(messageID))
                            // Update it
                            dataRow = (TableDataRow) dataRowCollection.get(messageID);
                        else
                            // Create a new TableDataRow object
                            dataRow = new TableDataRow();

                        // Sets Message content
                        dataRow.setMessage(canMessage);

                        // Sets readTimeStamp if need be
                        if (readTimeStamp)
                            dataRow.setRcvTime(rcvTime);
                        else
                            dataRow.setRcvTime((TPCANTimestamp)null);

                        // Sets counter
                        dataRow.setCounter(dataRow.getCounter() + 1);

                        //Put Message In the dataRowCollection
                        dataRowCollection.put(messageID, dataRow);
                    }
                 }
            }while(ret!= TPCANStatus.PCAN_ERROR_QRCVEMPTY || ret== TPCANStatus.PCAN_ERROR_OK );
            // Free local variables
            canMessage = null;
            rcvTime = null;
        }
        catch (Exception e)
        {
            System.out.println("CANReadThread Exception:" + e.getMessage());
            e.printStackTrace();
            System.exit(0);
        }
    }

    /**
     * Calls the PCANBasic Read Function according the readTimeStamp parameter
     *
     * @param handle The handle of a PCAN Channel
     */
    public void callAPIFunctionReadFd(TPCANHandle handle)
    {
        //Local variables
        TPCANMsgFD canMessage = null;
        TPCANTimestampFD rcvTime = null;
        TableDataRow dataRow = null;
        TPCANStatus ret;
        int messageID = 0;

        try
        {
            // Create new CAN Message
            canMessage = new TPCANMsgFD();
            // Create new TimeStamp object
            rcvTime = new TPCANTimestampFD();
            do
            {
                if (readTimeStamp) {
                    // We execute the "ReadFD" function of the PCANBasic
                    ret = pcanBasic.ReadFD(handle, canMessage, rcvTime);
                }
                else {
                    // We execute the "ReadFD" function of the PCANBasic
                    ret = pcanBasic.ReadFD(handle, canMessage, null);
                }

                //Process result
                if (ret == TPCANStatus.PCAN_ERROR_OK)
                {
                    //Gets CAN Message ID
                    messageID = canMessage.getID();
                    
                    // Searchs dataRowCollection contains CAN Message ID
                    if (dataRowCollection.containsKey(messageID))
                        // Update it
                        dataRow = (TableDataRow) dataRowCollection.get(messageID);
                    else
                        // Create a new TableDataRow object
                        dataRow = new TableDataRow();

                    // Sets Message content
                    dataRow.setMessage(canMessage);

                    // Sets readTimeStamp if need be
                    if (readTimeStamp)
                        dataRow.setRcvTime(rcvTime);
                    else
                        dataRow.setRcvTime((TPCANTimestampFD) null);

                    // Sets counter
                    dataRow.setCounter(dataRow.getCounter() + 1);

                    //Critical Area
                    synchronized (Application.token)
                    {
                        //Put Message In the dataRowCollection
                        dataRowCollection.put(messageID, dataRow);
                    }
                 }
            }while(ret!= TPCANStatus.PCAN_ERROR_QRCVEMPTY || ret== TPCANStatus.PCAN_ERROR_OK );
            // Free local variables
            canMessage = null;
            rcvTime = null;
        }
        catch (Exception e)
        {
            System.out.println("CANReadThread Exception:" + e.getMessage());
            e.printStackTrace();
            System.exit(0);
        }
    }
    // This function is called by the JNI library when a CAN Receive-Event is detected
    public void processRcvEvent(TPCANHandle channel)
    {
        for (ChannelItem item : connectedChannelCollection) {
            if (item.getHandle() == channel)
            {
                if (item.isCanFd())
                    // Process a PCANBasic readFD call
                    callAPIFunctionReadFd(channel);
                else                    
                    // Process a PCANBasic read call
                    callAPIFunctionRead(channel);
                return;
            }
        }
    }
}
