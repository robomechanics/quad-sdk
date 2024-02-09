/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * $Id: TableMessageThread.java 7377 2020-08-07 14:40:53Z Fabrice $
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

import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

/**
 * The TableMessageThread class extends Thread class and is used to process JTable display.
 *
 * @version 1.10
 * @LastChange $Date: 2020-08-07 16:40:53 +0200 (ven., 07 août 2020) $
 * @author Jonathan Urban/Uwe Wilhelm/Fabrice Vergnaud
 *
 * @Copyright (C) 1999-2014  PEAK-System Technik GmbH, Darmstadt
 * more Info at http://www.peak-system.com
 */
public class TableMessageThread extends Thread
{
    // Private Fields
    private JTable table;
    private HashMap<Integer, TableDataRow> data;

    /**
     * Default Constructor
     */
    public TableMessageThread()
    {
        try
        {
            jbInit();
        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
    }
    /**
     *
     * @param table JTable that the thread must refresh
     * @param data Reference to the Collection which store readed CAN Messages
     */
    public TableMessageThread(JTable table, HashMap<Integer, TableDataRow> data)
    {
        this.table = table;
        this.data = data;
    }

    /**
     * Starts thread process
     */
    public void run()
    {
        //Variables
        TableDataRow dataRow = null;
        //TPCANMsg msg = null;
        String msgIDStr = "";
        String msgLength = "";
        String msgType = "";
        String msgData = "";
        String blockData = "";
        String msgCount = "";
        String msgRcvTime = "";
        Object[] msgTableObect = null;
        HashMap<Integer, TableDataRow> datatmp;

        //Retrieve JTable Model
        DefaultTableModel model = (DefaultTableModel) table.getModel();
        int msgIndex = -1;
        while (true)
        {
            // make a copy of data (can be modified by external threads)
            synchronized (Application.token)
            {
                datatmp = new HashMap<Integer, TableDataRow>(data);
            }
            for (Object item : datatmp.values())
            {
                //Reset Variables Values
                msgIDStr = "";
                msgLength = "";
                msgType = "";
                msgData = "";
                msgCount = "";
                msgRcvTime = "";
                msgTableObect = null;
                msgIndex = -1;

                //Cast item in TableDataRow
                dataRow = (TableDataRow) item;

                //Get Type
                msgType = dataRow.getMsgType();
                //Message Length
                int msgLen = dataRow.getMsgLength();
                msgLength = String.valueOf(msgLen);
                //Message ID
                msgIDStr = Integer.toHexString(dataRow.getMsgId()) + "h";
                //Message Data
                byte[] d = dataRow.getMsgData();                
                for (int dataIndex = 0; dataIndex < msgLen; dataIndex++)
                {
                    blockData = Integer.toHexString(d[dataIndex] & 0xff);
                    if (blockData.length() == 1)
                        blockData = "0" + blockData;
                    msgData = msgData + blockData + " ";
                }
                //Message Count
                msgCount = String.valueOf(dataRow.getCounter());
                //Add Rcv Time If Need Be
                if (dataRow.getRcvTimeAsString() != null)
                    msgRcvTime = dataRow.getRcvTimeAsString();
                //Construct JTable Object
                msgTableObect = new Object[]{ msgType, msgIDStr, msgLength, msgData, msgCount, msgRcvTime};

                synchronized(Application.token)
                {
                    //Search Existing DataRow In Model
                    for (int i = 0; i < model.getRowCount(); i++)
                    {
                        if (model.getValueAt(i, 1).toString().equals(msgIDStr))
                        {
                            msgIndex = i;
                            break;
                        }
                    }
                    //Index not found
                    if (msgIndex == -1)
                        model.addRow(msgTableObect);
                    //Index was found
                    else
                    {
                        for(int i=0;i<6;i++)
                            model.setValueAt(msgTableObect[i], msgIndex, i);
                    }
                }
            }

            //Update UI
            table.repaint(table.getVisibleRect());

            try
            {
                //Thread.sleep(500);
                Thread.sleep(50);
            }
            catch (InterruptedException e)
            {
                System.out.println(e.getMessage());
                System.exit(0);
            }
        }
    }

    private void jbInit() throws Exception
    {
    }
}
