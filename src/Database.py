import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
from typing import Dict, Any

MapFieldValue = Dict[str, Any]
class Database:
    """
    Handles connection and synchronous operations with Google Cloud Firestore 
    using the firebase-admin Python SDK.
    """
    def __init__(self, project_id: str, credentials_path: str = None):
        """
        Initializes Firebase App and Firestore client.
        
        :param project_id: The ID of your Google Cloud project.
        :param credentials_path: Optional path to your service account JSON file.
                                 If None, uses Application Default Credentials.
        """
        self._db = None
        try:
            # Determine credentials: service account file path or default (gcloud auth)
            if credentials_path:
                cred = credentials.Certificate(credentials_path)
            else:
                # This is typically used when running on Google Cloud services
                # or after running 'gcloud auth application-default login' locally.
                cred = credentials.ApplicationDefault()

            # Initialize the app if it hasn't been already
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {'projectId': project_id})
            
            self._db = firestore.client()
            print("Firebase and Firestore initialized successfully.")
        except Exception as e:
            print(f"Error initializing Firebase. Check credentials and project ID. Error: {e}")
            self._db = None

    def set_document(self, collection: str, doc_id: str, data: MapFieldValue):
        """
        Sets (creates or completely overwrites) a document.
        
        :param collection: The name of the Firestore collection.
        :param doc_id: The ID of the document to set.
        :param data: The dictionary data to write to the document.
        """
        if not self._db:
            return

        try:
            self._db.collection(collection).document(doc_id).set(data)
            print(f"Document '{doc_id}' saved successfully in collection '{collection}'.")
        except Exception as e:
            print(f"Error saving document '{doc_id}': {e}")

    def update_document(self, collection: str, doc_id: str, updates: MapFieldValue):
        """
        Updates specific fields in an existing document without overwriting the whole document.
        
        :param collection: The name of the Firestore collection.
        :param doc_id: The ID of the document to update.
        :param updates: A dictionary of fields to update.
        """
        if not self._db:
            return

        try:
            
            self._db.collection(collection).document(doc_id).update(updates)
            #self._db.collection(collection).document(doc_id)
            
            print(f"Document '{doc_id}' updated successfully in collection '{collection}'.")
        except Exception as e:
            print(f"Error updating document '{doc_id}': {e}")

    def get_document(self, collection: str, doc_id: str) -> MapFieldValue:
        """
        Retrieves a document and returns its data as a dictionary.
        
        :param collection: The name of the Firestore collection.
        :param doc_id: The ID of the document to retrieve.
        :return: A dictionary containing the document data, or an empty dictionary if not found or on error.
        """
        if not self._db:
            return {}

        try:
            doc_ref = self._db.collection(collection).document(doc_id)
            doc = doc_ref.get()
            
            if doc.exists:
                print(f"Document '{doc_id}' read successfully.")
                return doc.to_dict()
            else:
                print(f"Document '{doc_id}' does not exist.")
                return {}
        except Exception as e:
            print(f"Error reading document '{doc_id}': {e}")
            return {}