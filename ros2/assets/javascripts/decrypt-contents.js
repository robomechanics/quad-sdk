/* encryptcontent/decrypt-contents.tpl.js */
// https://stackoverflow.com/a/50868276
function fromHex(hexString) {
    return new Uint8Array(hexString.match(/.{1,2}/g).map(byte => parseInt(byte, 16)));
}
// https://stackoverflow.com/a/41106346
function fromBase64(base64String) {
    return Uint8Array.from(atob(base64String), c => c.charCodeAt(0));
}

async function digestSHA256toBase64(message) {
  const data = new TextEncoder().encode(message);
  const hashBuffer = await crypto.subtle.digest("SHA-256", data);
  const hashArray = new Uint8Array(hashBuffer);
  const hashString = String.fromCharCode.apply(null, hashArray);
  return btoa(hashString);
};

function base64url_decode(input) {
    try {
        const binString = atob(input.replace(/-/g, '+').replace(/_/g, '/'));
        const binArray = Uint8Array.from(binString, (m) => m.codePointAt(0));
        return new TextDecoder().decode(binArray);
    }
    catch (err) {
        return false;
    }
}

/* Decrypts the key from the key bundle. */
async function decrypt_key(pass, iv_b64, ciphertext_b64, salt_b64) {
    const salt = fromBase64(salt_b64);
    const encPassword = new TextEncoder().encode(pass);
    const kdfkey = await window.crypto.subtle.importKey(
        "raw",
        encPassword,
        "PBKDF2",
        false,
        ["deriveKey"],
    );
    const wckey = await window.crypto.subtle.deriveKey(
        {
          name: "PBKDF2",
          salt,
          iterations: encryptcontent_obfuscate ? 1 : 100000,
          hash: "SHA-256",
        },
        kdfkey,
        { name: "AES-CBC", length: 256 },
        true,
        ["decrypt"],
    );
    const ciphertext = fromBase64(ciphertext_b64);
    const iv = fromBase64(iv_b64);
    try {
        const decrypted = await window.crypto.subtle.decrypt(
            {
                name: "AES-CBC",
                iv: iv
            },
            wckey,
            ciphertext
        );
        const keystore = JSON.parse(new TextDecoder().decode(decrypted));
        if (encryptcontent_id in keystore) {
            return keystore;
        } else {
            //id not found in keystore
            return false;
        }
    }
    catch (err) {
        // encoding failed; wrong key
        return false;
    }
};

/* Split key bundle and try to decrypt it */
async function decrypt_key_from_bundle(password, ciphertext_bundle, username) {
    // grab the ciphertext bundle and try to decrypt it
    let user, pass;
    let parts, keys, userhash;
    if (ciphertext_bundle) {
        if (username) {
            user = encodeURIComponent(username.toLowerCase());
            userhash = await digestSHA256toBase64(user);
        }
        for (let i = 0; i < ciphertext_bundle.length; i++) {
            pass = encodeURIComponent(password);
            parts = ciphertext_bundle[i].split(';');
            if (parts.length == 3) {
                keys = await decrypt_key(pass, parts[0], parts[1], parts[2]);
                if (keys) {
                    setCredentials(null, pass);
                    return keys;
                }
            } else if (parts.length == 4 && username) {
                if (parts[3] == userhash) {
                    keys = await decrypt_key(pass, parts[0], parts[1], parts[2]);
                    if (keys) {
                        setCredentials(user, pass);
                        return keys;
                    }
                }
            }
        }
    }
    return false;
};

/* Decrypts the content from the ciphertext bundle. */
async function decrypt_content(key, iv_b64, ciphertext_b64) {
    const rawKey = fromHex(key);
    const iv = fromBase64(iv_b64);
    const ciphertext = fromBase64(ciphertext_b64);
    try {
        const wckey = await window.crypto.subtle.importKey(           
            "raw",
            rawKey,                                                 
            "AES-CBC",
            true,
            ["decrypt"]
        );
        const decrypted = await window.crypto.subtle.decrypt(
            {
                name: "AES-CBC",
                iv: iv
            },
            wckey,
            ciphertext
        );
        const decoder = new TextDecoder();
        return decoder.decode(decrypted);
    }
    catch (err) {
        // encoding failed; wrong key
        return false;
    }
};

/* Split cyphertext bundle and try to decrypt it */
async function decrypt_content_from_bundle(key, ciphertext_bundle) {
    // grab the ciphertext bundle and try to decrypt it
    if (ciphertext_bundle) {
        let parts = ciphertext_bundle.split(';');
        if (parts.length == 2) {
            return await decrypt_content(key, parts[0], parts[1]);
        }
    }
    return false;
};

/* Save decrypted keystore to sessionStorage */
async function setKeys(keys_from_keystore) {
    for (const id in keys_from_keystore) {
        sessionStorage.setItem(id, keys_from_keystore[id]);
    }
};

/* Delete key with specific name in sessionStorage */
async function delItemName(key) {
    sessionStorage.removeItem(key);
};

async function getItemName(key) {
    return sessionStorage.getItem(key);
};
/* save username/password to sessionStorage/localStorage */
async function setCredentials(username, password) {
    localStorage.setItem('encryptcontent_credentials', JSON.stringify({'user': username, 'password': password}));
}

/* try to get username/password from sessionStorage/localStorage */
async function getCredentials(username_input, password_input) {
    const credentials = JSON.parse(localStorage.getItem('encryptcontent_credentials'));
    if (credentials && !encryptcontent_obfuscate) {
        if (credentials['user'] && username_input) {
            username_input.value = decodeURIComponent(credentials['user']);
        }
        if (credentials['password']) {
            password_input.value = decodeURIComponent(credentials['password']);
        }
        return true;
    } else {
        return false;
    }
}

/*remove username/password from localStorage */
async function delCredentials() {
    localStorage.removeItem('encryptcontent_credentials');
}

/* Reload scripts src after decryption process */
async function reload_js(src) {
    let script_src, script_tag, new_script_tag;
    let head = document.getElementsByTagName('head')[0];

    if (src.startsWith('#')) {
        script_tag = document.getElementById(src.substr(1));
        if (script_tag) {
            script_tag.remove();
            new_script_tag = document.createElement('script');
            if (script_tag.innerHTML) {
                new_script_tag.innerHTML = script_tag.innerHTML;
            }
            if (script_tag.src) {
                new_script_tag.src = script_tag.src;
            }
            if (script_tag.type) {
                new_script_tag.type = script_tag.type;
            }
            head.appendChild(new_script_tag);
        }
    } else {
        if (base_url == '.') {
            script_src = src;
        } else {
            script_src = base_url + '/' + src;
        }

        script_tag = document.querySelector('script[src="' + script_src + '"]');
        if (script_tag) {
            script_tag.remove();
            new_script_tag = document.createElement('script');
            new_script_tag.src = script_src;
            head.appendChild(new_script_tag);
        }
    }
};

/* Decrypt speficique html entry from mkdocs configuration */
async function decrypt_somethings(key, encrypted_something) {
    var html_item = '';
    for (const [name, tag] of Object.entries(encrypted_something)) {
        if (tag[1] == 'id') {
            html_item = [document.getElementById(name)];
        } else if (tag[1] == 'class') {
            html_item = document.getElementsByClassName(name);
        } else {
            console.log('WARNING: Unknow tag html found, check "encrypted_something" configuration.');
        }
        if (html_item[0]) {
            for (let i = 0; i < html_item.length; i++) {
                // grab the cipher bundle if something exist
                if (String(html_item[i].style.display).startsWith("none")) {
                    let content = await decrypt_content_from_bundle(key, html_item[i].innerHTML);
                    if (content !== false) {
                        // success; display the decrypted content
                        html_item[i].innerHTML = content;
                        html_item[i].style.display = null;
                        // any post processing on the decrypted content should be done here
                    }
                }
            }
        }
    }
    return html_item[0];
};

/* Decrypt content of a page */
async function decrypt_action(username_input, password_input, encrypted_content, decrypted_content, key_from_storage=false) {
    let key=false;
    let keys_from_keystore=false;

    let user=false;
    if (username_input) {
        user = username_input.value;
    }

    if (key_from_storage !== false) {
        key = key_from_storage;
    } else {
        keys_from_keystore = await decrypt_key_from_bundle(password_input.value, encryptcontent_keystore, user);
        if (keys_from_keystore) {
            key = keys_from_keystore[encryptcontent_id];
        }
    }

    let content = false;
    if (key) {
        content = await decrypt_content_from_bundle(key, encrypted_content.innerHTML);
    }
    if (content !== false) {
        // success; display the decrypted content
        decrypted_content.innerHTML = content;
        encrypted_content.parentNode.removeChild(encrypted_content);

        if (keys_from_keystore !== false) {
            return keys_from_keystore
        } else {
            return key
        }
    } else {
        return false
    }
};

async function decryptor_reaction(key_or_keys, password_input, decrypted_content, fallback_used=false) {
    if (key_or_keys) {
        let key;
        if (typeof key_or_keys === "object") {
            key = key_or_keys[encryptcontent_id];
            setKeys(key_or_keys);
        } else {
            key = key_or_keys;
        }

        // continue to decrypt others parts
        
        if (typeof inject_something !== 'undefined') {
            decrypted_content = await decrypt_somethings(key, inject_something);
        }
        if (typeof delete_something !== 'undefined') {
            let el = document.getElementById(delete_something)
            if (el) {
                el.remove();
            }
        }
        // any post processing on the decrypted content should be done here
        document$.next(document);
        let reload_scripts = ['https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.min.js'];
        for (let i = 0; i < reload_scripts.length; i++) { 
            await reload_js(reload_scripts[i]);
        }
        if (typeof theme_run_after_decryption !== 'undefined') {
            theme_run_after_decryption();
        }
        if (window.location.hash) { //jump to anchor if hash given after decryption
            window.location.href = window.location.hash;
        }
        //If we got keys then dispatch encryptcontent_event
        encryptcontent_done = true;
        window.dispatchEvent(encryptcontent_event);
    } else {
        // remove item on sessionStorage if decryption process fail (Invalid item)
        if (!fallback_used) {
            if (!encryptcontent_obfuscate) {
                // create HTML element for the inform message
                let mkdocs_decrypt_msg = document.getElementById('mkdocs-decrypt-msg');
                mkdocs_decrypt_msg.textContent = decryption_failure_message;
                password_input.value = '';
                password_input.focus();
            }
        }
        delItemName(encryptcontent_id);
    }
}

/* Trigger decryption process */
async function init_decryptor() {
    let username_input = document.getElementById('mkdocs-content-user');
    let password_input = document.getElementById('mkdocs-content-password');
    // adjust password field width to placeholder length
    //if (password_input.hasAttribute('placeholder')) {
    //    password_input.setAttribute('size', password_input.getAttribute('placeholder').length);
    //}
    let encrypted_content = document.getElementById('mkdocs-encrypted-content');
    let decrypted_content = document.getElementById('mkdocs-decrypted-content');
    let content_decrypted;
    /* If remember_keys is set, try to use sessionStorage item to decrypt content when page is loaded */
    let key_from_storage = await getItemName(encryptcontent_id);
    if (key_from_storage) {
        content_decrypted = await decrypt_action(
            username_input, password_input, encrypted_content, decrypted_content, key_from_storage
        );
        /* try to get username/password from sessionStorage */
        if (content_decrypted === false) {
            let got_credentials = await getCredentials(username_input, password_input);
            if (got_credentials) {
                content_decrypted = await decrypt_action(
                    username_input, password_input, encrypted_content, decrypted_content
                );
            }
        }
        decryptor_reaction(content_decrypted, password_input, decrypted_content, true);
    }
        else {
        let got_credentials = await getCredentials(username_input, password_input);
        if (got_credentials) {
            content_decrypted = await decrypt_action(
                username_input, password_input, encrypted_content, decrypted_content
            );
            decryptor_reaction(content_decrypted, password_input, decrypted_content, true);
        }
    }
    if (!content_decrypted) {
        //If nothing got decrypted, still dispatch encryptcontent_event
        encryptcontent_done = true;
        window.dispatchEvent(encryptcontent_event);
    }
    /* If password_button is set, try decrypt content when button is press */
    let decrypt_button = document.getElementById("mkdocs-decrypt-button");
    if (decrypt_button) {
        decrypt_button.onclick = async function(event) {
            event.preventDefault();
            content_decrypted = await decrypt_action(
                username_input, password_input, encrypted_content, decrypted_content
            );
            decryptor_reaction(content_decrypted, password_input, decrypted_content);
        };
    }
    /* Default, try decrypt content when key enter is press */
    password_input.addEventListener('keypress', async function(event) {
        if (event.key === "Enter") {
            event.preventDefault();
            content_decrypted = await decrypt_action(
                username_input, password_input, encrypted_content, decrypted_content
            );
            decryptor_reaction(content_decrypted, password_input, decrypted_content);
        }
    });
    decrypted_content.style.display = '';
}
if (typeof base_url === 'undefined') {
    var base_url = JSON.parse(document.getElementById('__config').textContent).base;
}
if (document.readyState === "loading") {
  // Loading hasn't finished yet
  document.addEventListener("DOMContentLoaded", init_decryptor);
} else {
  // `DOMContentLoaded` has already fired
  init_decryptor();
}
window["init_decryptor"] = init_decryptor;