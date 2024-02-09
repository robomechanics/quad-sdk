/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file resource.h
 * @brief PCAN Basic text resources
 * $Id: resource.h 13814 2022-01-18 11:16:55Z Fabrice $
 *
 * Copyright (C) 2001-2022  PEAK System-Technik GmbH <www.peak-system.com>
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
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Fabrice Vergnaud <f.vergnaud@peak-system.com>
 * Author:       Thomas Haber <thomas@toem.de>
 */

/* index of the language */
#define    IDS_STR_IND_LANG_DE          0
#define    IDS_STR_IND_LANG_EN          1
#define    IDS_STR_IND_LANG_ES          2
#define    IDS_STR_IND_LANG_FR          3
#define    IDS_STR_IND_LANG_IT          4
#define    IDS_STR_IND_LANG_NUM         5

/* index of the error's strings */
#define    IDS_STR_IND_ERR_OK           0
#define    IDS_STR_IND_ERR_XMTFULL      1
#define    IDS_STR_IND_ERR_OVERRUN      2
#define    IDS_STR_IND_ERR_BUSLIGHT     3
#define    IDS_STR_IND_ERR_BUSHEAVY     4
#define    IDS_STR_IND_ERR_BUSOFF       5
#define    IDS_STR_IND_ERR_ANYBUSERR    6
#define    IDS_STR_IND_ERR_QRCVEMPTY    7
#define    IDS_STR_IND_ERR_QOVERRUN     8
#define    IDS_STR_IND_ERR_QXMTFULL     9
#define    IDS_STR_IND_ERR_REGTEST      10
#define    IDS_STR_IND_ERR_NODRIVER     11
#define    IDS_STR_IND_ERR_RESOURCE     12
#define    IDS_STR_IND_ERR_ILLPARAMTYPE 13
#define    IDS_STR_IND_ERR_ILLPARAMVAL  14
#define    IDS_STR_IND_ERR_ILLHANDLE    15
#define    IDS_STR_IND_ERR_INITIALIZE   16
#define    IDS_STR_IND_ERR_UNKNOW       17
#define    IDS_STR_IND_ERR_HWINUSE      18
#define    IDS_STR_IND_ERR_NETINUSE     19
#define    IDS_STR_IND_ERR_ILLHW        20
#define    IDS_STR_IND_ERR_ILLNET       21
#define    IDS_STR_IND_ERR_ILLCLIENT    22
#define    IDS_STR_IND_ERR_ILLDATA	    23
#define    IDS_STR_IND_ERR_ILLOPERATION 24
#define    IDS_STR_IND_ERR_BUSPASSIVE   25
#define    IDS_STR_IND_ERR_CAUTION      26
#define    IDS_STR_IND_ERR_ILLMODE      27
#define    IDS_STR_IND_ERR_LANG_NUM     28

/* index of the log's strings */
#define    IDS_STR_IND_LOG_ENTER       0
#define    IDS_STR_IND_LOG_LEAVE       1
#define    IDS_STR_IND_LOG_PARAMS      2
#define    IDS_STR_IND_LOG_MSG         3
#define    IDS_STR_IND_LOG_EXCEPT      4
#define    IDS_STR_IND_LOG_CHANGEPATH  5
#define    IDS_STR_IND_LOG_CHANGEDPATH 6
#define    IDS_STR_IND_LOG_CUSTOM      7
#define    IDS_STR_IND_LOG_LANG_NUM    8

/******** GERMAN STRINGS ********/
/* Errors */
#define    IDS_STR_DE_ERR_OK          "Kein Fehler"
#define    IDS_STR_DE_ERR_XMTFULL     "Der Sendepuffer im Controller ist voll"
#define    IDS_STR_DE_ERR_OVERRUN     "Der CAN-Controller wurde zu spät gelesen"
#define    IDS_STR_DE_ERR_BUSLIGHT    "Bus-Fehler: Ein Fehlerzähler hat die ""light"" Obergrenze erreicht bzw. überschritten"
#define    IDS_STR_DE_ERR_BUSHEAVY    "Bus-Fehler: Ein Fehlerzähler hat die ""heavy""/""warning"""" Obergrenze erreicht bzw. überschritten"
#define    IDS_STR_DE_ERR_BUSOFF      "Bus-Fehler: Der CAN-Controller ist im BusOff- Zustand"
#define    IDS_STR_DE_ERR_ANYBUSERR   "Ein Busfehler ist aufgetreten"
#define    IDS_STR_DE_ERR_QRCVEMPTY   "Die Empfangsqueue ist leergelesen"
#define    IDS_STR_DE_ERR_QOVERRUN    "Die Empfangsqueue wurde zu spät gelesen"
#define    IDS_STR_DE_ERR_QXMTFULL    "Die Sendequeue ist voll"
#define    IDS_STR_DE_ERR_REGTEST     "Der RegisterTest des 82C200 schlug fehl"
#define    IDS_STR_DE_ERR_NODRIVER    "Der Treiber ist nicht geladen"
#define    IDS_STR_DE_ERR_RESOURCE    "Resource (FIFO, Client, Timeout) nicht erzeugbar"
#define    IDS_STR_DE_ERR_ILLPARAMTYPE "Parameter ist hier nicht erlaubt/anwendbar"
#define    IDS_STR_DE_ERR_ILLPARAMVAL "Der Parameterwert ist ungültig"
#define    IDS_STR_DE_ERR_ILLHANDLE   "Der Wert eines Handles (PCAN-Channel, PCAN-Hardware, PCAN-Net, PCAN-Client) ist ungültig"
#define    IDS_STR_DE_ERR_INITIALIZE  "Der PCAN Kanal ist nicht initialisiert oder der Initialisierungsvorgang ist fehlgeschlagen"
#define    IDS_STR_DE_ERR_UNKNOW      "Ein unbekannter Fehler ist aufgetreten"
#define    IDS_STR_DE_ERR_HWINUSE     "Die PCAN-Hardware ist bereits von einem PCAN-Netz belegt"
#define    IDS_STR_DE_ERR_NETINUSE    "Am PCAN-Netz ist bereits mindestens ein PCAN-Client angeschlossen"
#define    IDS_STR_DE_ERR_ILLHW       "Das Handle einer PCAN-Hardware ist ungültig"
#define    IDS_STR_DE_ERR_ILLNET      "Das Handle eines PCAN-Net ist ungültig"
#define    IDS_STR_DE_ERR_ILLCLIENT   "Das Handle eines PCAN-Client ist ungültig"
#define    IDS_STR_DE_ERR_ILLDATA     "Eine empfangene Nachricht ist ungültig"
#define    IDS_STR_DE_ERR_ILLOPERATION "Ein Vorgang ist aufgrund der aktuellen Konfiguration nicht zulässig"
#define    IDS_STR_DE_ERR_BUSPASSIVE  "Bus-Fehler: Der CAN-Controller ist im passiven Fehlerzustand"
#define    IDS_STR_DE_ERR_CAUTION     "Eine Operation wurde erfolgreich durchgeführt, aber Unregelmäßigkeiten wurden registriert"
#define    IDS_STR_DE_ERR_ILLMODE     "Der Treiberstatus ist für die versuchte Operation falsch"
/* log */
#define    IDS_STR_DE_LOG_ENTER       "[%04d]  EINGANG  '%s'"
#define    IDS_STR_DE_LOG_LEAVE       "[%04d]  AUSGANG  '%s' -   ERGEBNIS: 0x%02X"
#define    IDS_STR_DE_LOG_PARAMS      "[%04d]    PARAMETER von %s - %s"
#define    IDS_STR_DE_LOG_MSG         "[%04d]      KANAL  0x%02X (%s) %s"
#define    IDS_STR_DE_LOG_EXCEPT      "[%04d]  AUSNAHME  %d (0x%X) in %s: %s"
#define    IDS_STR_DE_LOG_CHANGEPATH  "[%04d]  PFAD  wird geändert zu '%s'"
#define    IDS_STR_DE_LOG_CHANGEDPATH "[%04d]  PFAD  wurde geändert von '%s'"
#define    IDS_STR_DE_LOG_CUSTOM      "[%04d]  BENUTZER  '%s'"
	
/******** ENGLISH STRINGS ********/
/* Errors */
#define    IDS_STR_EN_ERR_OK          "No Error"
#define    IDS_STR_EN_ERR_XMTFULL     "The transmit buffer in CAN controller is full"
#define    IDS_STR_EN_ERR_OVERRUN     "The CAN controller was read too late"
#define    IDS_STR_EN_ERR_BUSLIGHT    "Bus error: an error counter reached the 'light' limit"
#define    IDS_STR_EN_ERR_BUSHEAVY    "Bus error: an error counter reached the 'heavy'/'warning' limit"
#define    IDS_STR_EN_ERR_BUSOFF      "Bus error: the CAN controller is in bus-off state"
#define    IDS_STR_EN_ERR_ANYBUSERR   "A bus-error has occured"
#define    IDS_STR_EN_ERR_QRCVEMPTY   "The receive queue is empty"
#define    IDS_STR_EN_ERR_QOVERRUN    "The receive queue was read too late"
#define    IDS_STR_EN_ERR_QXMTFULL    "The transmit queue is full"
#define    IDS_STR_EN_ERR_REGTEST     "The test of the CAN controller hardware registers failed (no hardware found)"
#define    IDS_STR_EN_ERR_NODRIVER    "The driver is not loaded"
#define    IDS_STR_EN_ERR_RESOURCE    "A resource (FIFO, Client, timeout) cannot be created"
#define    IDS_STR_EN_ERR_ILLPARAMTYPE "An invalid parameter was specified"
#define    IDS_STR_EN_ERR_ILLPARAMVAL "A parameter contains an invalid value"
#define    IDS_STR_EN_ERR_ILLHANDLE   "The value of a handle (PCAN-Channel, PCAN-Hardware, PCAN-Net, PCAN-Client) is invalid"
#define    IDS_STR_EN_ERR_INITIALIZE  "A PCAN Channel has not been initialized yet or the initialization process has failed"
#define    IDS_STR_EN_ERR_UNKNOW      "An unknown error has occurred"
#define    IDS_STR_EN_ERR_HWINUSE     "The PCAN-Hardware is already being used by a PCAN-Net"
#define    IDS_STR_EN_ERR_NETINUSE    "A PCAN-Client is already connected to the PCAN-Net"
#define    IDS_STR_EN_ERR_ILLHW       "The value of a PCAN-Hardware handle is invalid"
#define    IDS_STR_EN_ERR_ILLNET      "The value of a PCAN-Net handle is invalid"
#define    IDS_STR_EN_ERR_ILLCLIENT   "The value of a PCAN-Client handle is invalid"
#define    IDS_STR_EN_ERR_ILLDATA     "A received message is invalid"
#define    IDS_STR_EN_ERR_ILLOPERATION	"An operation is not allowed due to the current configuration"
#define    IDS_STR_EN_ERR_BUSPASSIVE  "Bus error: The CAN controller is error passive"
#define    IDS_STR_EN_ERR_CAUTION     "An operation was successfully carried out, but irregularities were registered"
#define    IDS_STR_EN_ERR_ILLMODE     "The driver state is wrong for the attempted operation"
/* log */
#define    IDS_STR_EN_LOG_ENTER       "[%04d]  ENTRY  '%s'"
#define    IDS_STR_EN_LOG_LEAVE       "[%04d]  EXIT  '%s' -   RESULT: 0x%02X"
#define    IDS_STR_EN_LOG_PARAMS      "[%04d]    PARAMETERS of %s: %s"
#define    IDS_STR_EN_LOG_MSG         "[%04d]      CHANNEL  0x%02X (%s) %s"
#define    IDS_STR_EN_LOG_EXCEPT      "[%04d]  EXCEPTION  %d (0x%X) in %s: %s "
#define    IDS_STR_EN_LOG_CHANGEPATH  "[%04d]  PATH  changing to '%s'"
#define    IDS_STR_EN_LOG_CHANGEDPATH "[%04d]  PATH  changed from '%s'"
#define    IDS_STR_EN_LOG_CUSTOM      "[%04d]  USER  '%s'"
	
/******** SPANISH STRINGS ********/
/* Errors */
#define    IDS_STR_ES_ERR_OK          "Sin errores"
#define    IDS_STR_ES_ERR_XMTFULL     "El buffer de transmisión del controlador CAN está lleno"
#define    IDS_STR_ES_ERR_OVERRUN     "El controlador CAN se leyó muy tarde"
#define    IDS_STR_ES_ERR_BUSLIGHT    "Error en el Bus : Un contador de errores alcanzó el límite 'light'"
#define    IDS_STR_ES_ERR_BUSHEAVY    "Error en el Bus : Un contador de errores alcanzó el límite 'heavy'/'warning'"
#define    IDS_STR_ES_ERR_BUSOFF      "Error en el Bus : El controlador CAN se encuentra en el estado 'bus-off'"
#define    IDS_STR_ES_ERR_ANYBUSERR   "A ocurrido un error en el Bus"
#define    IDS_STR_ES_ERR_QRCVEMPTY   "La cola de recepción de mensajes está vacía"
#define    IDS_STR_ES_ERR_QOVERRUN    "La cola de recepción de mensajes fué leída demasiado tarde"
#define    IDS_STR_ES_ERR_QXMTFULL    "La cola de transmisión de mensajes está llena"
#define    IDS_STR_ES_ERR_REGTEST     "El test de registro de hardware del controador CAN falló (no se encontró ningún hardware)"
#define    IDS_STR_ES_ERR_NODRIVER    "El driver no se ha cargado"
#define    IDS_STR_ES_ERR_RESOURCE    "Algún recurso (FIFO, Client, timeout) no pudo ser creado"
#define    IDS_STR_ES_ERR_ILLPARAMTYPE "El parámetro dado es inválido"
#define    IDS_STR_ES_ERR_ILLPARAMVAL "El valor del parámetro dado es inválido"
#define    IDS_STR_ES_ERR_ILLHANDLE   "El valor del manejador (handle de: PCAN-Channel, PCAN-Hardware, PCAN-Net, PCAN-Client) es inválido"
#define    IDS_STR_ES_ERR_INITIALIZE  "El canal PCAN aún no ha sido inicializado o el proceso de inicialización ha fallado"
#define    IDS_STR_ES_ERR_UNKNOW      "Ha ocurrido un error desconocido"
#define    IDS_STR_ES_ERR_HWINUSE     "El dispositivo (PCAN-Hardware) ya está siendo utilizado por una red (PCAN-Net)"
#define    IDS_STR_ES_ERR_NETINUSE    "Un cliente (PCAN-Client) ya está conectado a la red (PCAN-Net)"
#define    IDS_STR_ES_ERR_ILLHW       "El valor del manejador (handle) de un dispositivo (PCAN-Hardware) es inválido"
#define    IDS_STR_ES_ERR_ILLNET      "El valor del manejador (handle) de una red (PCAN-Net) es inválido"
#define    IDS_STR_ES_ERR_ILLCLIENT   "El valor del manejador (handle) de un cliente (PCAN-Client) es inválido"
#define    IDS_STR_ES_ERR_ILLDATA     "Un mensaje recibido no es válido"
#define    IDS_STR_ES_ERR_ILLOPERATION "Una operación no está permitida debido a la configuración actual"
#define    IDS_STR_ES_ERR_BUSPASSIVE  "Error en el Bus: El controlador CAN se encuentra en el estado de error pasivo"
#define    IDS_STR_ES_ERR_CAUTION     "Una operación se realizó con éxito, sin embargo se registraron irregularidades"
#define    IDS_STR_ES_ERR_ILLMODE     "El estado del driver es incorrecto para la operación intentada"
/* log */	
#define    IDS_STR_ES_LOG_ENTER       "[%04d]  ENTRADA  '%s'"
#define    IDS_STR_ES_LOG_LEAVE       "[%04d]  SALIDA  %s -   RESULTADO: 0x%02X"
#define    IDS_STR_ES_LOG_PARAMS      "[%04d]    PARAMETROS de %s: %s"
#define    IDS_STR_ES_LOG_MSG         "[%04d]      CANAL  0x%02X (%s) %s"
#define    IDS_STR_ES_LOG_EXCEPT      "[%04d]  EXCEPCION  %d (0x%X) en %s: %s"
#define    IDS_STR_ES_LOG_CHANGEPATH  "[%04d]  DIRECTORIO  cambiará a '%s'"
#define    IDS_STR_ES_LOG_CHANGEDPATH "[%04d]  DIRECTORIO  cambió de '%s'"
#define    IDS_STR_ES_LOG_CUSTOM      "[%04d]  USUARIO  '%s'"
	
/******** FRENCH STRINGS ********/
/* Errors */
#define    IDS_STR_FR_ERR_OK          "Aucune erreur"
#define    IDS_STR_FR_ERR_XMTFULL     "Le buffer de transmission dans le contrôleur CAN est plein"
#define    IDS_STR_FR_ERR_OVERRUN     "Le contrôleur CAN a été lu trop tard"
#define    IDS_STR_FR_ERR_BUSLIGHT    "Erreur de bus: un compteur d'erreurs a atteint la limite 'light'"
#define    IDS_STR_FR_ERR_BUSHEAVY    "Erreur de bus: un compteur d'erreurs a atteint la limite 'heavy'/'warning'"
#define    IDS_STR_FR_ERR_BUSOFF      "Erreur de bus: le contrôleur CAN est passé à l'état 'bus-off'"
#define    IDS_STR_FR_ERR_ANYBUSERR   "Une erreur de bus a été détectée"
#define    IDS_STR_FR_ERR_QRCVEMPTY   "La queue de réception est vide "
#define    IDS_STR_FR_ERR_QOVERRUN    "La queue de réception a été lue trop tard"
#define    IDS_STR_FR_ERR_QXMTFULL    "La queue de transmission est pleine"
#define    IDS_STR_FR_ERR_REGTEST     "Le test du matériel du contrôleur CAN enregistré a échoué (aucun matériel détecté)"
#define    IDS_STR_FR_ERR_NODRIVER    "Le driver n'est pas chargé"
#define    IDS_STR_FR_ERR_RESOURCE    "Une ressource (FIFO, Client, timeout) ne peut pas être créée"
#define    IDS_STR_FR_ERR_ILLPARAMTYPE "Un paramètre invalide a été spécifié"
#define    IDS_STR_FR_ERR_ILLPARAMVAL "Un paramètre contient une valeur invalide"
#define    IDS_STR_FR_ERR_ILLHANDLE   "La valeur d'un Handle (PCAN-Channel, PCAN-Hardware, PCAN-Net, PCAN-Client) est incorrecte"
#define    IDS_STR_FR_ERR_INITIALIZE  "Un canal PCAN n'a pas encore été initialisé ou son initialisation a échoué"
#define    IDS_STR_FR_ERR_UNKNOW      "Une erreur inconnue s'est produite"
#define    IDS_STR_FR_ERR_HWINUSE     "Le PCAN-Hardware est déjà utilisé par un PCAN-Net"
#define    IDS_STR_FR_ERR_NETINUSE    "Un PCAN-Client est déjà connecté au PCAN-Net"
#define    IDS_STR_FR_ERR_ILLHW       "La valeur d'un PCAN-Hardware handle est incorrecte"
#define    IDS_STR_FR_ERR_ILLNET      "La valeur d'un PCAN-Net handle est incorrecte"
#define    IDS_STR_FR_ERR_ILLCLIENT   "La valeur d'un PCAN-Client handle est incorrecte"
#define    IDS_STR_FR_ERR_ILLDATA     "Un message reçu est invalide"
#define    IDS_STR_FR_ERR_ILLOPERATION "Une opération n'est pas autorisée en raison de la configuration actuelle"
#define    IDS_STR_FR_ERR_BUSPASSIVE  "Erreur de bus : le contrôleur CAN est en erreur passive"
#define    IDS_STR_FR_ERR_CAUTION     "Une opération a été menée avec succès, mais des irrégularités ont été enregistrées"
#define    IDS_STR_FR_ERR_ILLMODE     "L'état du driver est incorrect pour l'opération tentée"
/* log */
#define    IDS_STR_FR_LOG_ENTER       "[%04d]  ENTREE  '%s'"
#define    IDS_STR_FR_LOG_LEAVE       "[%04d]  SORTIE  '%s' -   RESULTAT: 0x%02X"
#define    IDS_STR_FR_LOG_PARAMS      "[%04d]    PARAMETRES  de %s: %s"
#define    IDS_STR_FR_LOG_MSG         "[%04d]      CANAL  0x%02X (%s) %s"
#define    IDS_STR_FR_LOG_EXCEPT      "[%04d]  EXCEPTION  %d (0x%X) en %s: %s"
#define    IDS_STR_FR_LOG_CHANGEPATH  "[%04d]  CHEMIN  est changé en '%s'"
#define    IDS_STR_FR_LOG_CHANGEDPATH "[%04d]  CHEMIN  a été modifié par '%s'"
#define    IDS_STR_FR_LOG_CUSTOM      "[%04d]  UTILISATEUR  '%s'"

/******** ITALIAN STRINGS ********/
/* Errors */
#define    IDS_STR_IT_ERR_OK          "Senza errori"
#define    IDS_STR_IT_ERR_XMTFULL     "Il buffer di trasmissione del controllore CAN è pieno"
#define    IDS_STR_IT_ERR_OVERRUN     "Il controllore CAN è stato letto troppo tardi"
#define    IDS_STR_IT_ERR_BUSLIGHT    "Errore di Bus: un contatore di errore ha raggiunto il limite 'light'"
#define    IDS_STR_IT_ERR_BUSHEAVY    "Errore di Bus: un contatore di errore ha raggiunto il limite 'heavy'/'warning'"
#define    IDS_STR_IT_ERR_BUSOFF      "Errore di Bus: il controllore CAN è in 'bus-off' stato"
#define    IDS_STR_IT_ERR_ANYBUSERR   "Vi è stato un errore di bus"
#define    IDS_STR_IT_ERR_QRCVEMPTY   "La coda di ricezione è vuota"
#define    IDS_STR_IT_ERR_QOVERRUN    "La coda di ricezione è stato letto troppo tardi"
#define    IDS_STR_IT_ERR_QXMTFULL    "La coda di trasmissione è piena"
#define    IDS_STR_IT_ERR_REGTEST     "La prova della registrazione di un controllore CAN hardware fallito (nessun hardware trovato)"
#define    IDS_STR_IT_ERR_NODRIVER    "Il driver non è caricato"
#define    IDS_STR_IT_ERR_RESOURCE    "Una risorsa (FIFO, Cliente, timeout) non può essere creato"
#define    IDS_STR_IT_ERR_ILLPARAMTYPE "Un parametro non valido è stato specificato"
#define    IDS_STR_IT_ERR_ILLPARAMVAL "Un parametro contiene un valore non valido"
#define    IDS_STR_IT_ERR_ILLHANDLE   "Un valore di handle (PCAN-Channel, PCAN-Hardware, PCAN-Net, PCAN-Client)  è sbagliato"
#define    IDS_STR_IT_ERR_INITIALIZE  "Un canale PCAN non è stata inizializzata, o ancora il processo di inizializzazione non è riuscito"
#define    IDS_STR_IT_ERR_UNKNOW      "Un errore sconosciuto si è verificato"
#define    IDS_STR_IT_ERR_HWINUSE     "L'PCAN-Hardware è già in uso da una PCAN-Net"
#define    IDS_STR_IT_ERR_NETINUSE    "Un PCAN-Client è già connesso alla PCAN-Net"
#define    IDS_STR_IT_ERR_ILLHW       "Il valore  del handle di un PCAN-Hardware è sbagliato"
#define    IDS_STR_IT_ERR_ILLNET      "Il valore  del handle di una PCAN-Net è sbagliato"
#define    IDS_STR_IT_ERR_ILLCLIENT   "Il valore  del handle di un PCAN-Client è sbagliato"
#define    IDS_STR_IT_ERR_ILLDATA     "Un messaggio ricevuto non è valido "
#define    IDS_STR_IT_ERR_ILLOPERATION "Un'operazione non è consentita a causa della configurazione corrente"
#define    IDS_STR_IT_ERR_BUSPASSIVE  "Errore di Bus: il controllore CAN è in condizione errore passiva"
#define    IDS_STR_IT_ERR_CAUTION     "Un'operazione è stata effettuata con successo, ma le irregolarità sono state registrate"
#define    IDS_STR_IT_ERR_ILLMODE     "Lo stato del driver non è corretto per l'operazione tentata"
/* log */
#define    IDS_STR_IT_LOG_ENTER       "[%04d]  ENTRATA  '%s'"
#define    IDS_STR_IT_LOG_LEAVE       "[%04d]  USCITA  %s -   RISULTATO: 0x%02X"
#define    IDS_STR_IT_LOG_PARAMS      "[%04d]    PARAMETRI di %s: %s"
#define    IDS_STR_IT_LOG_MSG         "[%04d]      CANALE  0x%02X (%s) %s"
#define    IDS_STR_IT_LOG_EXCEPT      "[%04d]  ECCEZIONE  %d (0x%X) in %s: %s"
#define    IDS_STR_IT_LOG_CHANGEPATH  "[%04d]  PATH  è stato cambiato in '%s'"
#define    IDS_STR_IT_LOG_CHANGEDPATH "[%04d]  PATH  è stato cambiato da '%s'"
#define    IDS_STR_IT_LOG_CUSTOM      "[%04d]  UTENTE  '%s'"


const char* resource[IDS_STR_IND_LANG_NUM][IDS_STR_IND_ERR_LANG_NUM] = {
	{
		IDS_STR_DE_ERR_OK,
		IDS_STR_DE_ERR_XMTFULL,
		IDS_STR_DE_ERR_OVERRUN,
		IDS_STR_DE_ERR_BUSLIGHT,
		IDS_STR_DE_ERR_BUSHEAVY,
		IDS_STR_DE_ERR_BUSOFF,
		IDS_STR_DE_ERR_ANYBUSERR,
		IDS_STR_DE_ERR_QRCVEMPTY,
		IDS_STR_DE_ERR_QOVERRUN,
		IDS_STR_DE_ERR_QXMTFULL,
		IDS_STR_DE_ERR_REGTEST,
		IDS_STR_DE_ERR_NODRIVER,
		IDS_STR_DE_ERR_RESOURCE,
		IDS_STR_DE_ERR_ILLPARAMTYPE,
		IDS_STR_DE_ERR_ILLPARAMVAL,
		IDS_STR_DE_ERR_ILLHANDLE,
		IDS_STR_DE_ERR_INITIALIZE,
		IDS_STR_DE_ERR_UNKNOW,
		IDS_STR_DE_ERR_HWINUSE,
		IDS_STR_DE_ERR_NETINUSE,
		IDS_STR_DE_ERR_ILLHW,
		IDS_STR_DE_ERR_ILLNET,
		IDS_STR_DE_ERR_ILLCLIENT,
		IDS_STR_DE_ERR_ILLDATA,
		IDS_STR_DE_ERR_ILLOPERATION,
		IDS_STR_DE_ERR_BUSPASSIVE,
		IDS_STR_DE_ERR_CAUTION,
		IDS_STR_DE_ERR_ILLMODE
	}, {
		IDS_STR_EN_ERR_OK,
		IDS_STR_EN_ERR_XMTFULL,
		IDS_STR_EN_ERR_OVERRUN,
		IDS_STR_EN_ERR_BUSLIGHT,
		IDS_STR_EN_ERR_BUSHEAVY,
		IDS_STR_EN_ERR_BUSOFF,
		IDS_STR_EN_ERR_ANYBUSERR,
		IDS_STR_EN_ERR_QRCVEMPTY,
		IDS_STR_EN_ERR_QOVERRUN,
		IDS_STR_EN_ERR_QXMTFULL,
		IDS_STR_EN_ERR_REGTEST,
		IDS_STR_EN_ERR_NODRIVER,
		IDS_STR_EN_ERR_RESOURCE,
		IDS_STR_EN_ERR_ILLPARAMTYPE,
		IDS_STR_EN_ERR_ILLPARAMVAL,
		IDS_STR_EN_ERR_ILLHANDLE,
		IDS_STR_EN_ERR_INITIALIZE,
		IDS_STR_EN_ERR_UNKNOW,
		IDS_STR_EN_ERR_HWINUSE,
		IDS_STR_EN_ERR_NETINUSE,
		IDS_STR_EN_ERR_ILLHW,
		IDS_STR_EN_ERR_ILLNET,
		IDS_STR_EN_ERR_ILLCLIENT,
		IDS_STR_EN_ERR_ILLDATA,
		IDS_STR_EN_ERR_ILLOPERATION,
		IDS_STR_EN_ERR_BUSPASSIVE,
		IDS_STR_EN_ERR_CAUTION,
		IDS_STR_EN_ERR_ILLMODE
	}, {
		IDS_STR_ES_ERR_OK,
		IDS_STR_ES_ERR_XMTFULL,
		IDS_STR_ES_ERR_OVERRUN,
		IDS_STR_ES_ERR_BUSLIGHT,
		IDS_STR_ES_ERR_BUSHEAVY,
		IDS_STR_ES_ERR_BUSOFF,
		IDS_STR_ES_ERR_ANYBUSERR,
		IDS_STR_ES_ERR_QRCVEMPTY,
		IDS_STR_ES_ERR_QOVERRUN,
		IDS_STR_ES_ERR_QXMTFULL,
		IDS_STR_ES_ERR_REGTEST,
		IDS_STR_ES_ERR_NODRIVER,
		IDS_STR_ES_ERR_RESOURCE,
		IDS_STR_ES_ERR_ILLPARAMTYPE,
		IDS_STR_ES_ERR_ILLPARAMVAL,
		IDS_STR_ES_ERR_ILLHANDLE,
		IDS_STR_ES_ERR_INITIALIZE,
		IDS_STR_ES_ERR_UNKNOW,
		IDS_STR_ES_ERR_HWINUSE,
		IDS_STR_ES_ERR_NETINUSE,
		IDS_STR_ES_ERR_ILLHW,
		IDS_STR_ES_ERR_ILLNET,
		IDS_STR_ES_ERR_ILLCLIENT,
		IDS_STR_ES_ERR_ILLDATA,
		IDS_STR_ES_ERR_ILLOPERATION,
		IDS_STR_ES_ERR_BUSPASSIVE,
		IDS_STR_ES_ERR_CAUTION,
		IDS_STR_ES_ERR_ILLMODE
	}, {
		IDS_STR_FR_ERR_OK,
		IDS_STR_FR_ERR_XMTFULL,
		IDS_STR_FR_ERR_OVERRUN,
		IDS_STR_FR_ERR_BUSLIGHT,
		IDS_STR_FR_ERR_BUSHEAVY,
		IDS_STR_FR_ERR_BUSOFF,
		IDS_STR_FR_ERR_ANYBUSERR,
		IDS_STR_FR_ERR_QRCVEMPTY,
		IDS_STR_FR_ERR_QOVERRUN,
		IDS_STR_FR_ERR_QXMTFULL,
		IDS_STR_FR_ERR_REGTEST,
		IDS_STR_FR_ERR_NODRIVER,
		IDS_STR_FR_ERR_RESOURCE,
		IDS_STR_FR_ERR_ILLPARAMTYPE,
		IDS_STR_FR_ERR_ILLPARAMVAL,
		IDS_STR_FR_ERR_ILLHANDLE,
		IDS_STR_FR_ERR_INITIALIZE,
		IDS_STR_FR_ERR_UNKNOW,
		IDS_STR_FR_ERR_HWINUSE,
		IDS_STR_FR_ERR_NETINUSE,
		IDS_STR_FR_ERR_ILLHW,
		IDS_STR_FR_ERR_ILLNET,
		IDS_STR_FR_ERR_ILLCLIENT,
		IDS_STR_FR_ERR_ILLDATA,
		IDS_STR_FR_ERR_ILLOPERATION,
		IDS_STR_FR_ERR_BUSPASSIVE,
		IDS_STR_FR_ERR_CAUTION,
		IDS_STR_FR_ERR_ILLMODE
	}, {
		IDS_STR_IT_ERR_OK,
		IDS_STR_IT_ERR_XMTFULL,
		IDS_STR_IT_ERR_OVERRUN,
		IDS_STR_IT_ERR_BUSLIGHT,
		IDS_STR_IT_ERR_BUSHEAVY,
		IDS_STR_IT_ERR_BUSOFF,
		IDS_STR_IT_ERR_ANYBUSERR,
		IDS_STR_IT_ERR_QRCVEMPTY,
		IDS_STR_IT_ERR_QOVERRUN,
		IDS_STR_IT_ERR_QXMTFULL,
		IDS_STR_IT_ERR_REGTEST,
		IDS_STR_IT_ERR_NODRIVER,
		IDS_STR_IT_ERR_RESOURCE,
		IDS_STR_IT_ERR_ILLPARAMTYPE,
		IDS_STR_IT_ERR_ILLPARAMVAL,
		IDS_STR_IT_ERR_ILLHANDLE,
		IDS_STR_IT_ERR_INITIALIZE,
		IDS_STR_IT_ERR_UNKNOW,
		IDS_STR_IT_ERR_HWINUSE,
		IDS_STR_IT_ERR_NETINUSE,
		IDS_STR_IT_ERR_ILLHW,
		IDS_STR_IT_ERR_ILLNET,
		IDS_STR_IT_ERR_ILLCLIENT,
		IDS_STR_IT_ERR_ILLDATA,
		IDS_STR_IT_ERR_ILLOPERATION,
		IDS_STR_IT_ERR_BUSPASSIVE,
		IDS_STR_IT_ERR_CAUTION,
		IDS_STR_IT_ERR_ILLMODE
	} };

const char* resource_log[IDS_STR_IND_LANG_NUM][IDS_STR_IND_LOG_LANG_NUM] = {
	{
		IDS_STR_DE_LOG_ENTER,
		IDS_STR_DE_LOG_LEAVE,
		IDS_STR_DE_LOG_PARAMS,
		IDS_STR_DE_LOG_MSG,
		IDS_STR_DE_LOG_EXCEPT,
		IDS_STR_DE_LOG_CHANGEPATH,
		IDS_STR_DE_LOG_CHANGEDPATH,
		IDS_STR_DE_LOG_CUSTOM
	}, {
		IDS_STR_EN_LOG_ENTER,
		IDS_STR_EN_LOG_LEAVE,
		IDS_STR_EN_LOG_PARAMS,
		IDS_STR_EN_LOG_MSG,
		IDS_STR_EN_LOG_EXCEPT,
		IDS_STR_EN_LOG_CHANGEPATH,
		IDS_STR_EN_LOG_CHANGEDPATH,
		IDS_STR_EN_LOG_CUSTOM
	}, {
		IDS_STR_ES_LOG_ENTER,
		IDS_STR_ES_LOG_LEAVE,
		IDS_STR_ES_LOG_PARAMS,
		IDS_STR_ES_LOG_MSG,
		IDS_STR_ES_LOG_EXCEPT,
		IDS_STR_ES_LOG_CHANGEPATH,
		IDS_STR_ES_LOG_CHANGEDPATH,
		IDS_STR_ES_LOG_CUSTOM
	}, {
		IDS_STR_FR_LOG_ENTER,
		IDS_STR_FR_LOG_LEAVE,
		IDS_STR_FR_LOG_PARAMS,
		IDS_STR_FR_LOG_MSG,
		IDS_STR_FR_LOG_EXCEPT,
		IDS_STR_FR_LOG_CHANGEPATH,
		IDS_STR_FR_LOG_CHANGEDPATH,
		IDS_STR_FR_LOG_CUSTOM
	}, {
		IDS_STR_IT_LOG_ENTER,
		IDS_STR_IT_LOG_LEAVE,
		IDS_STR_IT_LOG_PARAMS,
		IDS_STR_IT_LOG_MSG,
		IDS_STR_IT_LOG_EXCEPT,
		IDS_STR_IT_LOG_CHANGEPATH,
		IDS_STR_IT_LOG_CHANGEDPATH,
		IDS_STR_IT_LOG_CUSTOM
	} };
